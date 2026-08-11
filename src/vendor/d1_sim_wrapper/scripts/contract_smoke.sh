#!/usr/bin/env bash
# End-to-end acceptance test of the D1-Max vendor-interface wrapper against a
# RUNNING sim (ros2 launch d1_sim_wrapper d1_vendor_sim.launch.py).
# Run with the same ROS_DOMAIN_ID as the sim (source env.bash first).
set -u

MAP_ROOT="${MAP_ROOT:-$HOME/ign_dog_maps/history_map}"
CMD_START="${CMD_START:-0}"
CMD_STOP_SAVE="${CMD_STOP_SAVE:-1}"
PASS=0; FAIL=0

check() { # check <label> <ok:0|nonzero>
  if [ "$2" -eq 0 ]; then echo "PASS  $1"; PASS=$((PASS+1));
  else echo "FAIL  $1"; FAIL=$((FAIL+1)); fi
}

wait_slam_state() { # wait_slam_state <state> <timeout_sec>
  timeout "$2" bash -c \
    "until ros2 topic echo /pub_slam_state --once --field data.data 2>/dev/null | head -1 | grep -qx '$1'; do sleep 0.5; done"
}

wait_loc_status() { # wait_loc_status <status> <timeout_sec>
  timeout "$2" bash -c \
    "until ros2 topic echo /localization_state --once --field status 2>/dev/null | head -1 | grep -qx '$1'; do sleep 0.5; done"
}

wait_nav_state() { # wait_nav_state <state regex> <timeout_sec>
  timeout "$2" bash -c \
    "until ros2 topic echo /navigation_state --once --field state 2>/dev/null | head -1 | grep -qxE '$1'; do sleep 0.3; done"
}

echo "== 1. start mapping =="
OUT=$(ros2 service call /slam_state_service robots_dog_msgs/srv/MapState \
  "{mapping_type: 0, arc_id: 0, map_path_prefix: '', data: $CMD_START}" 2>&1)
echo "$OUT" | grep -q "success=True"; check "start_mapping accepted" $?
wait_slam_state 3 10; check "slam state -> MAPPING(3)" $?

echo "== 2. move the dog to grow the map =="
timeout 6 ros2 topic pub -r 10 /cmd_vel geometry_msgs/msg/Twist '{linear: {x: 0.25}}' >/dev/null 2>&1

echo "== 3. stop mapping -> save =="
OUT=$(ros2 service call /slam_state_service robots_dog_msgs/srv/MapState \
  "{mapping_type: 0, arc_id: 0, map_path_prefix: '', data: $CMD_STOP_SAVE}" 2>&1)
echo "$OUT" | grep -q "success=True"; check "stop_mapping accepted" $?
wait_slam_state 6 15; check "slam state -> SAVED(6)" $?

MAP_DIR=$(ls -dt "$MAP_ROOT"/*/ 2>/dev/null | head -1)
[ -n "$MAP_DIR" ] && [ -f "$MAP_DIR/map.yaml" ] && [ -f "$MAP_DIR/map.pgm" ] && [ -f "$MAP_DIR/map.pcd" ]
check "map files exist in $MAP_DIR" $?

echo "== 4. load map =="
OUT=$(ros2 service call /load_map_service robots_dog_msgs/srv/LoadMap \
  "{pcd_path: '${MAP_DIR}map.pcd'}" 2>&1)
echo "$OUT" | grep -q "success=True"; check "load_map accepted" $?
wait_loc_status 3 20; check "localization -> NORMAL(3)" $?

echo "== 5. read pose, send one POI 0.5m back along the mapped corridor =="
# the goal must be in KNOWN free space: aim back into the area the dog just
# drove through while mapping (ahead of the robot is unknown space).
POSE=$(ros2 topic echo /localization_info --once --field pos 2>/dev/null)
echo "current pos: $POSE"
X=$(echo "$POSE" | sed -n 's/^x: //p'); Y=$(echo "$POSE" | sed -n 's/^y: //p')
GX=$(python3 -c "print(float('$X') - 0.5)"); GY=$(python3 -c "print(float('$Y'))")

ros2 topic pub --once -w 1 /start_navigation robots_dog_msgs/msg/StartNavigation \
  "{header: {frame_id: map}, cmd: 1, secondary_cmd: 0, function_id: 4,
    map_path: '${MAP_DIR}map.yaml', reference_path: '', target_id: -1,
    goals: [{position: {x: $GX, y: $GY, z: 0.0},
             orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}],
    speed: {x: 0.5, y: 0.5, z: 1.5},
    goal_tolerance: {x: 0.0, y: 0.0, theta: 0.0}, extra_info: ''}" >/dev/null 2>&1

wait_nav_state '(1|2)' 10; check "navigation -> INITIALIZING/ACTIVE" $?
wait_nav_state '5' 120;   check "navigation -> SUCCEEDED(5)" $?

echo "== 6. negative: goal in unknown space =="
ros2 topic pub --once -w 1 /start_navigation robots_dog_msgs/msg/StartNavigation \
  "{header: {frame_id: map}, cmd: 1, secondary_cmd: 0, function_id: 4,
    map_path: '${MAP_DIR}map.yaml', reference_path: '', target_id: -1,
    goals: [{position: {x: 500.0, y: 500.0, z: 0.0},
             orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}],
    speed: {x: 0.5, y: 0.5, z: 1.5},
    goal_tolerance: {x: 0.0, y: 0.0, theta: 0.0}, extra_info: ''}" >/dev/null 2>&1
wait_nav_state '6' 10; check "out-of-map goal -> FAILED(6)" $?

echo
echo "== RESULT: $PASS passed, $FAIL failed =="
exit $FAIL
