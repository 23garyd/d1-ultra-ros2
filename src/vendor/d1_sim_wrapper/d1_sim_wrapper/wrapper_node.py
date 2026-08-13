"""D1-Max vendor-interface wrapper node.

Exposes the 9-interface robots_dog_msgs contract on top of the sim's
cartographer (/map, map->odom TF) and Nav2 (/navigate_to_pose):

  server  /slam_state_service   robots_dog_msgs/srv/MapState
  pub     /pub_slam_state       robots_dog_msgs/msg/SlamState        2 Hz
  server  /load_map_service     robots_dog_msgs/srv/LoadMap
  pub     /localization_state   robots_dog_msgs/msg/LocalizationState 10 Hz
  pub     /localization_info    robots_dog_msgs/msg/Localization     10 Hz
  pub     /navigo/ms/cmn/intf/map  nav_msgs/msg/OccupancyGrid        on load/save
  sub     /start_navigation     robots_dog_msgs/msg/StartNavigation
  pub     /navigation_state     robots_dog_msgs/msg/NavigationState  20 Hz (BEST_EFFORT)
  pub     /diagnostics/nav_error_report robots_dog_msgs/msg/NavigationErrorReport
                                 on new NavExecutor.last_error (RELIABLE)

QoS profiles replicate the vendor capture (see plan / bag metadata).
"""
import math
import os
import threading

import rclpy
from rclpy.action import ActionClient
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.duration import Duration
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import (DurabilityPolicy, HistoryPolicy, QoSProfile,
                       ReliabilityPolicy)
from rclpy.time import Time

from geometry_msgs.msg import Pose
from nav_msgs.msg import OccupancyGrid, Odometry
from nav2_msgs.action import NavigateToPose
from tf2_ros import (Buffer, ConnectivityException, ExtrapolationException,
                     LookupException, TransformListener)

from robots_dog_msgs.msg import (Localization, LocalizationState,
                                 NavigationErrorReport, NavigationState,
                                 SlamState, StartNavigation)
from robots_dog_msgs.srv import LoadMap, MapState

from . import map_store
from .localization_monitor import DESCRIPTIONS, LocalizationMonitor, LocStatus
from .nav_executor import NavExecutor, NavState
from .slam_session import SlamSession, SlamStateCode


def _quat_normalized(q, tol: float = 1e-3) -> bool:
    n = math.sqrt(q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w)
    return abs(n - 1.0) < tol


def _quat_to_rpy(q):
    sinr_cosp = 2.0 * (q.w * q.x + q.y * q.z)
    cosr_cosp = 1.0 - 2.0 * (q.x * q.x + q.y * q.y)
    roll = math.atan2(sinr_cosp, cosr_cosp)
    sinp = 2.0 * (q.w * q.y - q.z * q.x)
    pitch = math.copysign(math.pi / 2.0, sinp) if abs(sinp) >= 1.0 else math.asin(sinp)
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    yaw = math.atan2(siny_cosp, cosy_cosp)
    return roll, pitch, yaw


def _duration_msg(seconds: float):
    return Duration(seconds=max(0.0, seconds)).to_msg()


class D1SimWrapper(Node):

    def __init__(self):
        super().__init__('d1_sim_wrapper')

        self.declare_parameter('cmd_start', 0)
        self.declare_parameter('cmd_stop_save', 1)
        self.declare_parameter('cmd_cancel', 2)
        self.declare_parameter('map_root', '~/ign_dog_maps/history_map')
        self.declare_parameter('tf_stale_sec', 2.0)
        self.declare_parameter('loc_error_sec', 10.0)
        self.declare_parameter('terminal_hold_sec', 5.0)
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('base_frame', 'base_link')
        p = self.get_parameter
        self._cmd_start = p('cmd_start').value
        self._cmd_stop_save = p('cmd_stop_save').value
        self._cmd_cancel = p('cmd_cancel').value
        self._map_root = os.path.expanduser(p('map_root').value)
        self._map_frame = p('map_frame').value
        self._base_frame = p('base_frame').value

        self._slam = SlamSession()
        self._loc = LocalizationMonitor(
            tf_stale_sec=p('tf_stale_sec').value,
            loc_error_sec=p('loc_error_sec').value)
        self._nav = NavExecutor(terminal_hold_sec=p('terminal_hold_sec').value)

        self._lock = threading.Lock()  # guards state read by Group-B timers
        self._live_grid = None         # cached cartographer /map
        self._loaded_grid = None       # GridData of the loaded saved map
        self._loaded_grid_msg = None   # OccupancyGrid published on the vendor topic
        self._loaded_map_dir = None
        self._latest_odom = None
        self._nav2_goal_handle = None
        self._loc_status = LocStatus.INIT

        self._grp_state = MutuallyExclusiveCallbackGroup()  # Group A: mutations
        self._grp_read = MutuallyExclusiveCallbackGroup()   # Group B: publishers

        reliable10 = QoSProfile(reliability=ReliabilityPolicy.RELIABLE,
                                durability=DurabilityPolicy.VOLATILE,
                                history=HistoryPolicy.KEEP_LAST, depth=10)
        best_effort10 = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                                   durability=DurabilityPolicy.VOLATILE,
                                   history=HistoryPolicy.KEEP_LAST, depth=10)
        map_qos = QoSProfile(history=HistoryPolicy.KEEP_ALL,
                             reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.VOLATILE, depth=1)
        carto_map_qos = QoSProfile(reliability=ReliabilityPolicy.RELIABLE,
                                   durability=DurabilityPolicy.TRANSIENT_LOCAL,
                                   history=HistoryPolicy.KEEP_LAST, depth=1)

        self._pub_slam = self.create_publisher(SlamState, '/pub_slam_state', reliable10)
        self._pub_loc_state = self.create_publisher(
            LocalizationState, '/localization_state', reliable10)
        self._pub_loc_info = self.create_publisher(
            Localization, '/localization_info', reliable10)
        self._pub_nav_state = self.create_publisher(
            NavigationState, '/navigation_state', best_effort10)
        self._pub_map = self.create_publisher(
            OccupancyGrid, '/navigo/ms/cmn/intf/map', map_qos)
        self._error_report = self.create_publisher(
            NavigationErrorReport, "/diagnostics/nav_error_report",
            reliable10, callback_group=self._grp_read)
        self._reported_error = None

        self.create_subscription(OccupancyGrid, '/map', self._on_map,
                                 carto_map_qos, callback_group=self._grp_read)
        self.create_subscription(Odometry, '/odom', self._on_odom,
                                 QoSProfile(depth=10), callback_group=self._grp_read)
        self.create_subscription(StartNavigation, '/start_navigation',
                                 self._on_start_navigation, reliable10,
                                 callback_group=self._grp_state)

        self.create_service(MapState, '/slam_state_service',
                            self._on_slam_state_service,
                            callback_group=self._grp_state)
        self.create_service(LoadMap, '/load_map_service',
                            self._on_load_map_service,
                            callback_group=self._grp_state)

        self._nav2_client = ActionClient(self, NavigateToPose, 'navigate_to_pose',
                                         callback_group=self._grp_state)

        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self, spin_thread=True)

        self.create_timer(1.0 / 2.0, self._publish_slam_state,
                          callback_group=self._grp_read)
        self.create_timer(1.0 / 10.0, self._publish_localization,
                          callback_group=self._grp_read)
        self.create_timer(1.0 / 20.0, self._publish_navigation_state,
                          callback_group=self._grp_read)

        self.get_logger().info(
            f'd1_sim_wrapper up: map_root={self._map_root} '
            f'cmd_start={self._cmd_start} cmd_stop_save={self._cmd_stop_save} '
            f'cmd_cancel={self._cmd_cancel}')

    # ---------------- helpers ----------------

    def _now_sec(self) -> float:
        return self.get_clock().now().nanoseconds / 1e9

    def _tf_pose_and_age(self):
        """(transform, age_sec) of map->base_link, or (None, None)."""
        try:
            t = self._tf_buffer.lookup_transform(
                self._map_frame, self._base_frame, Time())
        except (LookupException, ConnectivityException, ExtrapolationException):
            return None, None
        age = (self.get_clock().now() - Time.from_msg(t.header.stamp)).nanoseconds / 1e9
        return t, age

    def _grid_from_msg(self, msg: OccupancyGrid) -> map_store.GridData:
        q = msg.info.origin.orientation
        _, _, yaw = _quat_to_rpy(q)
        return map_store.GridData(
            width=msg.info.width, height=msg.info.height,
            resolution=msg.info.resolution,
            origin_x=msg.info.origin.position.x,
            origin_y=msg.info.origin.position.y,
            origin_yaw=yaw, cells=list(msg.data))

    def _msg_from_grid(self, grid: map_store.GridData) -> OccupancyGrid:
        msg = OccupancyGrid()
        msg.header.frame_id = self._map_frame
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.info.resolution = float(grid.resolution)
        msg.info.width = grid.width
        msg.info.height = grid.height
        msg.info.origin.position.x = float(grid.origin_x)
        msg.info.origin.position.y = float(grid.origin_y)
        x, y, z, w = map_store.yaw_to_quaternion(grid.origin_yaw)
        msg.info.origin.orientation.x = x
        msg.info.origin.orientation.y = y
        msg.info.origin.orientation.z = z
        msg.info.origin.orientation.w = w
        msg.data = [v if -1 <= v <= 100 else -1 for v in grid.cells]
        return msg

    # ---------------- subscriptions ----------------

    def _on_map(self, msg: OccupancyGrid) -> None:
        with self._lock:
            self._live_grid = msg

    def _on_odom(self, msg: Odometry) -> None:
        with self._lock:
            self._latest_odom = msg

    # ---------------- mapping lifecycle ----------------

    def _on_slam_state_service(self, req: MapState.Request,
                               resp: MapState.Response) -> MapState.Response:
        if req.mapping_type != 0:
            resp.success = False
            resp.message = f'mapping_type {req.mapping_type} not supported (only 0: normal)'
            return resp

        if req.data == self._cmd_start:
            ok, msg = self._slam.start(navigation_active=self._nav.busy)
            resp.success, resp.message = ok, msg
        elif req.data == self._cmd_stop_save:
            ok, msg = self._slam.begin_save()
            if ok:
                self._publish_slam_state()  # SAVING(5) observable before SAVED(6)
                ok, msg = self._do_save_map(req.map_path_prefix)
            resp.success, resp.message = ok, msg
        elif req.data == self._cmd_cancel:
            ok, msg = self._slam.cancel()
            resp.success, resp.message = ok, msg
        else:
            resp.success = False
            resp.message = (f'unknown command data={req.data}; wrapper accepts '
                            f'start={self._cmd_start} stop_save={self._cmd_stop_save} '
                            f'cancel={self._cmd_cancel}')
        self.get_logger().info(
            f'/slam_state_service data={req.data} -> {resp.success} ({resp.message})')
        return resp

    def _do_save_map(self, path_prefix: str):
        with self._lock:
            grid_msg = self._live_grid
        if grid_msg is None:
            self._slam.mark_save_failed('no /map received from cartographer yet')
            return False, self._slam.error_msg
        try:
            grid = self._grid_from_msg(grid_msg)
            map_id = map_store.generate_map_id()
            root = os.path.expanduser(path_prefix) if path_prefix else self._map_root
            map_dir = map_store.save_map(grid, root, map_id)
            for name in map_store.REQUIRED_FILES:  # verify before reporting SAVED
                if not os.path.isfile(os.path.join(map_dir, name)):
                    raise IOError(f'{name} missing after save')
        except Exception as e:  # noqa: BLE001 — any save failure must not kill the node
            self._slam.mark_save_failed(str(e))
            return False, self._slam.error_msg
        self._slam.mark_saved(map_dir)
        self._pub_map.publish(grid_msg)  # fresh map observable, vendor-style
        self.get_logger().info(f'map saved: {map_dir}')
        return True, f'map saved to {map_dir}'

    def _publish_slam_state(self) -> None:
        msg = SlamState()
        msg.timestamp = self.get_clock().now().to_msg()
        msg.arc_id = 255  # captured normal-mapping value
        msg.arc_detection_flag = False
        msg.map_path_prefix = self._slam.map_path_prefix
        msg.data.data = int(self._slam.state)
        msg.error_code = self._slam.error_code
        msg.error_msg = self._slam.error_msg
        t, _ = self._tf_pose_and_age()
        if t is not None and self._slam.state == SlamStateCode.MAPPING:
            msg.pose.position.x = t.transform.translation.x
            msg.pose.position.y = t.transform.translation.y
            msg.pose.position.z = t.transform.translation.z
            msg.pose.orientation = t.transform.rotation
        self._pub_slam.publish(msg)

    # ---------------- map load ----------------

    def _on_load_map_service(self, req: LoadMap.Request,
                             resp: LoadMap.Response) -> LoadMap.Response:
        if self._slam.mapping_active:
            resp.success = False
            resp.message = 'rejected: mapping in progress'
            return resp
        ok, result = map_store.validate_map_dir(req.pcd_path)
        if not ok:
            resp.success = False
            resp.message = result
            self.get_logger().warn(f'/load_map_service rejected: {result}')
            return resp
        map_dir = result
        try:
            grid = map_store.load_map(map_dir)
        except Exception as e:  # noqa: BLE001
            resp.success = False
            resp.message = f'failed to load map: {e}'
            self.get_logger().error(resp.message)
            return resp

        grid_msg = self._msg_from_grid(grid)
        with self._lock:
            self._loaded_grid = grid
            self._loaded_grid_msg = grid_msg
            self._loaded_map_dir = map_dir
        self._pub_map.publish(grid_msg)
        self._loc.notify_map_loaded(self._now_sec())
        resp.success = True
        resp.message = f'map loaded from {map_dir}'
        self.get_logger().info(resp.message)
        return resp

    # ---------------- localization ----------------

    def _publish_localization(self) -> None:
        now = self._now_sec()
        t, age = self._tf_pose_and_age()
        status = self._loc.evaluate(now, age)
        prev = self._loc_status
        self._loc_status = status

        if status in (LocStatus.ERROR, LocStatus.LOST) and prev not in (
                LocStatus.ERROR, LocStatus.LOST):
            if self._nav.on_localization_lost(now):
                self.get_logger().warn(
                    'localization lost during active goal -> failing goal')
                self._cancel_nav2_goal()

        state_msg = LocalizationState()
        state_msg.header.stamp = self.get_clock().now().to_msg()
        state_msg.status = int(status)
        state_msg.rate = self._loc.rate_for(status)
        state_msg.description = DESCRIPTIONS[status]
        self._pub_loc_state.publish(state_msg)

        info = Localization()
        info.header.stamp = self.get_clock().now().to_msg()
        info.header.frame_id = self._map_frame
        info.type = 'loc_state'
        info.status = self._loc.info_status_for(status)
        info.coord_type = 0
        if t is not None:
            info.pos.x = t.transform.translation.x
            info.pos.y = t.transform.translation.y
            info.pos.z = t.transform.translation.z
            r, p_, y = _quat_to_rpy(t.transform.rotation)
            info.rpy.x, info.rpy.y, info.rpy.z = r, p_, y
        with self._lock:
            odom = self._latest_odom
        if odom is not None:
            tw = odom.twist.twist
            info.vel.x, info.vel.y, info.vel.z = tw.linear.x, tw.linear.y, tw.linear.z
            info.gyro.x, info.gyro.y, info.gyro.z = tw.angular.x, tw.angular.y, tw.angular.z
            info.speed = math.sqrt(tw.linear.x ** 2 + tw.linear.y ** 2)
        self._pub_loc_info.publish(info)

    # ---------------- navigation ----------------

    def _on_start_navigation(self, msg: StartNavigation) -> None:
        now = self._now_sec()
        if msg.cmd == StartNavigation.CMD_START:
            err = self._validate_start(msg)
            if err:
                self.get_logger().warn(f'/start_navigation rejected: {err}')
                goal = msg.goals[0] if msg.goals else Pose()
                self._nav.fail_rejected_start(now, msg.function_id, goal)
                return
            gen = self._nav.begin_start(msg.goals[0], msg.function_id)
            self.get_logger().info(
                f'starting goal gen={gen} -> ({msg.goals[0].position.x:.2f}, '
                f'{msg.goals[0].position.y:.2f}) speed={msg.speed} '
                f'tolerance=({msg.goal_tolerance.x}, {msg.goal_tolerance.y}, '
                f'{msg.goal_tolerance.theta})')
            self._send_nav2_goal(msg.goals[0], gen)
        elif msg.cmd == StartNavigation.CMD_PAUSE:
            if self._nav.request_pause():
                self._cancel_nav2_goal()
            else:
                self.get_logger().warn('pause ignored: no active goal')
        elif msg.cmd == StartNavigation.CMD_CONTINUE:
            gen = self._nav.begin_continue()
            if gen is None:
                self.get_logger().warn('continue ignored: not paused')
            else:
                self._send_nav2_goal(self._nav.goal_pose, gen)
        elif msg.cmd == StartNavigation.CMD_STOP:
            if self._nav.request_stop(now):
                self._cancel_nav2_goal()
        else:
            self.get_logger().warn(f'unknown /start_navigation cmd={msg.cmd}')

    def _validate_start(self, msg: StartNavigation):
        """Contract §7 publish preconditions; returns error string or None."""
        with self._lock:
            loaded_dir = self._loaded_map_dir
            grid = self._loaded_grid
        if loaded_dir is None:
            return 'no map loaded (call /load_map_service first)'
        if not os.path.isfile(os.path.join(loaded_dir, 'map.yaml')):
            return 'loaded map.yaml no longer exists'
        if msg.map_path and os.path.normpath(os.path.expanduser(msg.map_path)) != \
                os.path.normpath(os.path.join(loaded_dir, 'map.yaml')):
            return (f'map_path {msg.map_path!r} does not match loaded map '
                    f'{os.path.join(loaded_dir, "map.yaml")!r}')
        if self._loc_status != LocStatus.NORMAL:
            return f'localization not ready (status {int(self._loc_status)}, need 3)'
        if self._nav.busy:
            return f'another goal is active (state {int(self._nav.state)})'
        if self._slam.mapping_active:
            return 'mapping in progress'
        if msg.function_id != StartNavigation.FUNCTION_PATROL:
            return f'function_id {msg.function_id} not supported (only 4: patrol)'
        if msg.header.frame_id and msg.header.frame_id != self._map_frame:
            return f'goal frame {msg.header.frame_id!r} must be {self._map_frame!r}'
        if len(msg.goals) != 1:
            return f'exactly one goal required, got {len(msg.goals)}'
        goal = msg.goals[0]
        if not _quat_normalized(goal.orientation):
            return 'goal quaternion is not normalized'
        cell = grid.cell_at_world(goal.position.x, goal.position.y)
        if cell is None:
            return (f'goal ({goal.position.x:.2f}, {goal.position.y:.2f}) '
                    'is outside the loaded map')
        if cell == -1:
            return 'goal cell is unknown space'
        if cell >= 100 * map_store.OCCUPIED_THRESH:
            return 'goal cell is occupied'
        return None

    def _send_nav2_goal(self, pose: Pose, generation: int) -> None:
        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = self._map_frame
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose = pose

        def on_feedback(fb_msg, gen=generation):
            fb = fb_msg.feedback
            self._nav.on_feedback(
                gen, fb.distance_remaining,
                fb.estimated_time_remaining.sec + fb.estimated_time_remaining.nanosec / 1e9,
                fb.navigation_time.sec + fb.navigation_time.nanosec / 1e9)

        if not self._nav2_client.server_is_ready():
            self.get_logger().error('nav2 /navigate_to_pose server not ready')
            self._nav.on_goal_rejected(generation, self._now_sec())
            return
        send_future = self._nav2_client.send_goal_async(goal, feedback_callback=on_feedback)
        send_future.add_done_callback(
            lambda fut, gen=generation: self._on_nav2_response(fut, gen))

    def _on_nav2_response(self, future, generation: int) -> None:
        handle = future.result()
        if handle is None or not handle.accepted:
            self.get_logger().error('nav2 rejected the goal')
            self._nav.on_goal_rejected(generation, self._now_sec())
            return
        if self._nav.is_current(generation):
            self._nav2_goal_handle = handle
        self._nav.on_goal_accepted(generation)
        handle.get_result_async().add_done_callback(
            lambda fut, gen=generation: self._on_nav2_result(fut, gen))

    def _on_nav2_result(self, future, generation: int) -> None:
        status = future.result().status
        self.get_logger().info(f'nav2 result gen={generation} status={status}')
        self._nav.on_result(generation, status, self._now_sec())

    def _cancel_nav2_goal(self) -> None:
        if self._nav2_goal_handle is not None:
            self._nav2_goal_handle.cancel_goal_async()

    def _publish_navigation_state(self) -> None:
        state = self._nav.tick(self._now_sec())
        msg = NavigationState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self._map_frame
        msg.state = int(state)
        msg.function_id = self._nav.function_id
        t, _ = self._tf_pose_and_age()
        if t is not None:
            msg.current_pose.position.x = t.transform.translation.x
            msg.current_pose.position.y = t.transform.translation.y
            msg.current_pose.position.z = t.transform.translation.z
            msg.current_pose.orientation = t.transform.rotation
        if self._nav.goal_pose is not None:
            msg.current_goal = self._nav.goal_pose
        fb = self._nav.feedback
        msg.navigation_time = _duration_msg(fb.nav_time_sec)
        msg.estimated_time_remaining_to_current = _duration_msg(fb.eta_sec)
        msg.estimated_time_remaining_to_final = _duration_msg(fb.eta_sec)
        msg.remaining_distance_to_current = fb.distance_remaining
        msg.remaining_distance_to_final = fb.distance_remaining
        self._pub_nav_state.publish(msg)
        self._maybe_publish_error()

    def _maybe_publish_error(self) -> None:
        error = self._nav.last_error
        if error is None or error is self._reported_error:
            return
        report = NavigationErrorReport()
        report.correlation_id = str(self._nav.last_terminal or 0)
        report.scope = error.scope
        report.stamp = self.get_clock().now().to_msg()
        report.primary.code = error.code
        report.primary.message = error.message
        report.primary.stamp = report.stamp
        self._error_report.publish(report)
        self._reported_error = error


def main(args=None):
    rclpy.init(args=args)
    node = D1SimWrapper()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
