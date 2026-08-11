# D1-Max Vendor-Interface Wrapper for ign_robot_dog Sim

## Context

The cloud SDK task executor (remote patrol backend) drives a real Agibot D1-Max dog through a fixed set of proprietary ROS 2 interfaces (`robots_dog_msgs` topics/services), documented in `~/Public/d1-max-minimal-wrapper-interface-contract.md` and learned from real captures in `~/Public/d1-max-captures` and `~/Public/Agibot_D1_Max/ros2_capture`. The goal: make the ign_robot_dog Gazebo sim expose **the same 8 interfaces with the same types, semantics, and QoS**, so the executor can run its full mapping → load-map → localize → one-POI-at-a-time patrol workflow against the sim unchanged.

Decisions made with user:
- **Full contract** — all 8 interfaces (mapping lifecycle + navigation).
- **ROS_DOMAIN_ID=24, default DDS** (rmw_fastrtps); no Zenoh.
- **Real map save; SLAM-based localization** — `stop_mapping` really saves cartographer's map to a history_map folder; `load_map` validates + publishes the saved grid; the robot keeps localizing via live cartographer (no AMCL).
- **One Python node with modular internals** — new `d1_sim_wrapper` ament_python package.

Key discovery: a **complete buildable `robots_dog_msgs` package already exists** at `/home/demo/Public/tracer-mini-d1-fastlio/robots_dog_msgs/` (7 msgs + 6 srvs, with contract-freeze sha256 manifest + golden `ros2 interface show` dumps + pytest checks). We copy it into this workspace for wire compatibility — we do NOT hand-recreate definitions.

## Source-of-truth references

- Contract: `/home/demo/Public/d1-max-minimal-wrapper-interface-contract.md`
- Vendor QoS (from bag `/home/demo/Public/d1-max-captures/d1_mapping_capture_20260810_122204/metadata.yaml`):
  - `/pub_slam_state`, `/localization_state`, `/localization_info`: KEEP_LAST(10), RELIABLE, VOLATILE
  - `/navigation_state`: KEEP_LAST(10), **BEST_EFFORT**, VOLATILE
  - `/navigo/ms/cmn/intf/map`: KEEP_ALL, RELIABLE, VOLATILE (published only on load/save, not latched)
  - `/start_navigation` (we subscribe): RELIABLE, VOLATILE
- Observed rates (bag, 258 s): slam_state ~2 Hz, localization_state/info ~10 Hz, navigation_state ~20 Hz.
- Msg/srv definitions: `/home/demo/Public/tracer-mini-d1-fastlio/robots_dog_msgs/{msg,srv}/`
- Existing sim facts (explored):
  - Map topic `/map` from `cartographer_occupancy_grid_node_sim` (TRANSIENT_LOCAL); `map→odom` TF from cartographer (`dog.lua: published_frame="odom"`); no map_server/AMCL; no save/load plumbing.
  - Nav2 action `/navigate_to_pose` (bt_navigator); lifecycle manager `dog_lifecycle_manager`; nav2 cmd path `/raw_cmd_vel → velocity_smoother → /cmd_vel`.
  - Robot pose: TF `map→base_link`; odom topic `/odom` (EKF output).
  - `MapState.data` command enum is unknown even on the real robot → wrapper takes it as parameters.

## New packages

### 1. `src/vendor/robots_dog_msgs/` — copied verbatim
Copy from `/home/demo/Public/tracer-mini-d1-fastlio/robots_dog_msgs/` including `contract/` (interface_manifest.json + golden dumps) and `test/`. No modifications — the contract tests guarantee wire identity.

### 2. `src/vendor/d1_sim_wrapper/` — ament_python
```
d1_sim_wrapper/
  package.xml, setup.py, setup.cfg, resource/
  d1_sim_wrapper/
    __init__.py
    wrapper_node.py          # node wiring: pubs/subs/services/timers, param declaration
    slam_session.py          # mapping state machine READY(2)→MAPPING(3)→SAVING(5)→SAVED(6)
    map_store.py             # history_map dirs, map_id gen (YYYY_MM_DD_HH_MM_SS), save /map→pgm+yaml,
                             # placeholder map.pcd, validate/resolve pcd_path→map dir, load yaml+pgm→OccupancyGrid
    localization_monitor.py  # /localization_state + /localization_info synthesis from TF+/odom
    nav_executor.py          # /start_navigation → Nav2 NavigateToPose client; /navigation_state machine
  launch/d1_vendor_sim.launch.py   # includes sim_ign_dog/d1_gazebo_sim_dog.launch.py + wrapper node
  launch/wrapper_only.launch.py    # wrapper alone (sim already running)
  config/wrapper_params.yaml
  test/  (pytest unit tests for the three state machines with fake clock/TF)
  scripts/contract_smoke.sh        # end-to-end acceptance script (ros2 cli driven)
```

## Behavior per interface (contract §-by-§)

1. **`/slam_state_service` (MapState, server)** — params `cmd_start` (default 0), `cmd_stop_save` (default 1), `cmd_cancel` (default 2); unknown `data` → `success=false` + message. Start: reject if navigating or already mapping; enter MAPPING. Stop: enter SAVING, run async map save, then SAVED. `mapping_type != 0` (ARC) → rejected (out of scope, per contract).
2. **`/pub_slam_state` (2 Hz)** — publishes current state; `error_code=0`, `error_msg` "Mapping state is normal."-style strings matching captures; `map_path_prefix` set to the saved dir once SAVED. Publish SAVED only after files verified on disk (contract: state 6 + error_code 0 is the success signal).
3. **Map save (validated approach)** — the wrapper keeps a persistent TRANSIENT_LOCAL subscriber on `/map` and caches the latest OccupancyGrid; on stop it writes the cache **in-process** (no `map_saver_cli` subprocess — order-independent of cartographer's latched publisher, instant, and we need the PGM code for load anyway). Writer: PGM `P5` maxval 255; pixel 254=free/0=occupied/205=unknown; rows written top-down (reverse of grid row order); YAML with `image: map.pgm`, `mode: trinary`, `resolution`, `origin: [x,y,yaw]`, `negate: 0`, `occupied_thresh: 0.65`, `free_thresh: 0.25`. `map_id = time.strftime("%Y_%m_%d_%H_%M_%S")` (wall clock, vendor format), dir `<map_root>/<map_id>/`; placeholder `map.pcd` (valid PCD header, empty cloud). `map_root` param, default `~/ign_dog_maps/history_map` (settable to `/ota/alg_data/map/history_map` for path-identical testing). SAVING(5) is published on transition even though the save is fast; SAVED(6) only after files verified on disk.
4. **`/load_map_service` (LoadMap, server)** — `pcd_path` → parent dir must contain `map.yaml`, `map.pgm`, `map.pcd` (contract: validate all required files). On success: load yaml+pgm → OccupancyGrid (frame `map`), publish on `/navigo/ms/cmn/intf/map`, mark map loaded, drive localization 1→2→3. Response `success=false` with reason on any validation failure. Reject while mapping. PGM loader (validated): P5/trinary only (reject P2/scale with clear error — we only load what we save); comment/whitespace-tolerant header tokenizer; `p=(maxval-v)/maxval` when `negate==0`; `p>occupied_thresh→100`, `p<free_thresh→0`, else `-1`; **flip rows** (PGM row 0 = top, grid index 0 = bottom-left); `origin[2]` yaw → quaternion; PyYAML only (already a ROS dep).
5. **`/localization_state` (10 Hz)** — status machine: 0 Init (no map loaded) → 1 Map loading → 2 Initial localization → 3 Continuous/normal (map loaded AND TF `map→base_link` fresh within `tf_stale_sec`, default 2.0) ; TF stale while loaded → 6 Lost, prolonged (> `loc_error_sec`) → 4 Error. `rate=1.0` when 3, else 0.0; `description` strings mirror captured ones ("continuous loc: CONTINUOUS_NORMAL", "loc lost: CONTINUOUS_LOST").
6. **`/localization_info` (10 Hz)** — `header.frame_id="map"`, `type="loc_state"`, `coord_type=0`; `pos`/`rpy` from TF `map→base_link` (quaternion→euler); `vel`,`gyro`,`speed` from `/odom`; `acc` zeros. `status`: 0 before load, 3 normal, 4 lost (aligned with monitor).
7. **`/navigo/ms/cmn/intf/map`** — vendor QoS (KEEP_ALL, RELIABLE, VOLATILE). Published on: load_map success (saved grid) and mapping save completion (fresh grid). Optional param `map_republish_sec` (default 0 = vendor-faithful no republish).
8. **`/start_navigation` (subscriber)** — precondition gate before acting on `cmd=1` (contract §7 list): map loaded + yaml exists + localization status 3 + fresh loc/nav data + no active goal + frame `map` + normalized quaternion (|‖q‖−1| < 1e-3) + exactly 1 goal + goal inside map bounds and cell free. On violation: log + publish one FAILED(6) NavigationState burst (observability), stay Standby.
   - `cmd=1` start → NavigateToPose goal to Nav2; state INITIALIZING(1) → ACTIVE(2) on feedback.
   - `cmd=2` pause → cancel Nav2 goal, retain goal + poi context, state PAUSE(3).
   - `cmd=3` continue → resend retained goal, → INITIALIZING/ACTIVE.
   - `cmd=4` stop → cancel, state CANCELLED(4), then Standby after retention.
   - `function_id` echoed into `/navigation_state`; only 4 (patrol) accepted for v1.
   - `speed`/`goal_tolerance`: accepted, logged; not applied to Nav2 in v1 (stretch: set controller max_vel via SetParameters).
9. **`/navigation_state` (20 Hz, BEST_EFFORT)** — mirrors executor state; `current_pose` from TF, `current_goal`, `navigation_time` + `estimated_time_remaining_*` + `remaining_distance_*` from NavigateToPose feedback (`distance_remaining`, `estimated_time_remaining`, `navigation_time`); final==current for single goal. Terminal mapping: Nav2 SUCCEEDED→5, ABORTED→6, CANCELED→4. **Terminal retention**: hold terminal state for `terminal_hold_sec` (default 5.0) before returning to Standby (vendor returned to standby after terminal; contract requires the terminal be observable — internally also latch `last_terminal` until next start).
10. **Cross-interface rules** — localization 4/6 during ACTIVE → cancel Nav2 goal, publish FAILED (matches captured trace); mapping commands rejected while navigating and vice-versa (contract checklist "prevents concurrent mapping and navigation").

## Runtime / env

- Everything (sim + wrapper) runs under `ROS_DOMAIN_ID=24`. `d1_vendor_sim.launch.py` cannot set the domain of an already-started daemon-side context reliably → provide `env.bash` at repo root (`export ROS_DOMAIN_ID=24`) + README note; launch file also passes `ROS_DOMAIN_ID` via `SetEnvironmentVariable` for child processes as belt-and-braces.
- `use_sim_time:=true` for the wrapper (mandatory — TF freshness compares node clock vs transform stamps; wall clock would make everything look stale); header stamps normalized (`0 ≤ nanosec < 1e9` — contract calls out a vendor bug here; rclpy Time does this natively).
- Domain-24 footnotes for README: every terminal needs `export ROS_DOMAIN_ID=24` (provide `env.bash`), and after switching domains run `ros2 daemon stop` once (CLI daemon caches discovery per domain). `ros_gz_bridge` honors the domain; ign-transport side is domain-agnostic — no issue.

## Implementation notes (validated by design review)

- **Concurrency**: MultiThreadedExecutor + two MutuallyExclusiveCallbackGroups. Group A (state mutation): both service servers, `/start_navigation` sub, action-client response/feedback/result callbacks — serialized, no locks. Group B (read-only): the 2/10/20 Hz publisher timers + `/odom`/`/map` subs, reading atomic state snapshots. No ReentrantCallbackGroup (invites races). Golden rule: no synchronous future waits inside any callback — chain `add_done_callback` throughout.
- **NavigateToPose client (Humble)**: `send_goal_async(goal, feedback_callback=...)` → done-callback gets ClientGoalHandle → check `.accepted` → `get_result_async()`. Result status: `GoalStatus.STATUS_SUCCEEDED==4`→5, `STATUS_CANCELED==5`→4 or Pause (see below), `STATUS_ABORTED==6`→6. Feedback gives `distance_remaining`, `estimated_time_remaining`, `navigation_time` directly. Guards: tag each goal with a monotonically increasing generation id and ignore stale callbacks; ignore feedback arriving after a terminal state.
- **Pause/continue**: cancel+resend is the accepted Humble pattern (BT navigator handles cancel cleanly; velocity_smoother decays cmd_vel). Enter PAUSE(3) only after the cancel result confirms; store the original PoseStamped; continue = fresh `send_goal_async` (planner replans from current pose). Map the CANCELED result to Pause(3) vs Cancelled(4) by *why we canceled* (intent flag), relying on the generation-id guard. Do NOT use the `/speed_limit`=0 trick (progress checker aborts after ~10 s).
- **TF freshness**: `tf2_ros.Buffer` + `TransformListener(buffer, node, spin_thread=True)` (own thread, no executor contention). Staleness = `now() - lookup_transform('map','base_link', Time()).header.stamp`; static links don't cap the common time, so this measures exactly the dynamic edges (cartographer map→odom + EKF). Threshold ~1.0–2.0 s. Catch LookupException/ConnectivityException/ExtrapolationException → NOT_LOCALIZED, never crash the 10 Hz timer. Gazebo pause freezes age — document, acceptable.
- **QoS (rclpy)**: `QoSProfile(reliability=BEST_EFFORT, durability=VOLATILE, history=KEEP_LAST, depth=10)` for `/navigation_state`; `QoSProfile(history=KEEP_ALL, reliability=RELIABLE, durability=VOLATILE, depth=1)` for the map topic (KEEP_ALL publishes fine in rclpy; one-shot publish can't block).

## Files modified (existing)

- **None of the working launch/config files change.** The wrapper ships its own launch that *includes* `sim_ign_dog/launch/d1_gazebo_sim_dog.launch.py`.
- `README.md`: new section "D1-Max vendor interface wrapper" (how to launch, env, map_root, smoke test).

## Verification

1. `colcon build` clean; `colcon test --packages-select robots_dog_msgs` → contract-freeze tests pass (interface sha256 + golden dumps).
2. Unit tests (`pytest` in d1_sim_wrapper): slam state machine transitions incl. save failure; localization gating incl. stale-TF → 6/4; nav executor incl. pause/continue/stop, terminal retention, precondition rejections.
3. End-to-end `scripts/contract_smoke.sh` against the running sim (domain 24):
   - `ros2 service call /slam_state_service ... data: <cmd_start>` → `/pub_slam_state` shows 3.
   - drive via `/cmd_vel` a few seconds; `... data: <cmd_stop_save>` → 5 then 6; assert `<map_root>/<id>/{map.yaml,map.pgm,map.pcd}` exist.
   - `ros2 service call /load_map_service "{pcd_path: <id>/map.pcd}"` → success; `/localization_state` reaches 3; one OccupancyGrid arrives on `/navigo/ms/cmn/intf/map`.
   - publish one `/start_navigation` (cmd 1, function 4, goal in free space) → `/navigation_state` 1→2→5 and robot moves in Gazebo.
   - negative: goal in occupied cell → FAILED burst, no motion; stop mid-goal → 4.
4. QoS verification: `ros2 topic info -v` on the 5 published topics matches the vendor bag's profiles (esp. BEST_EFFORT on `/navigation_state`).

## Out of scope (v1)

ARC mapping (`mapping_type=1`), `/get_slam_state_service`, `/srv/event`, tracking/exploration function_ids, applying `speed`/`goal_tolerance` to Nav2, Zenoh RMW, multi-goal lists.

---

## Implementation outcome (2026-08-11)

Implemented as designed in commit `新增D1-Max云端接口仿真wrapper`. Verification: 28 unit
tests pass; `scripts/contract_smoke.sh` passes 10/10 end-to-end against the live sim on
ROS_DOMAIN_ID=24 (map → save → load → localize → POI SUCCEEDED → negative goal rejected);
QoS on published topics matches the vendor bag.

Deviations from the plan discovered during implementation:
- `robots_dog_msgs/test/test_contract.py` was dropped from the copied package — it tests the
  extraction tool that only exists in the tracer-mini-d1-fastlio source repo. Wire identity is
  still guaranteed by `test_generated_interfaces.py` against `contract/` golden dumps.
- `FREE_THRESH` is 0.196 (nav2 map_saver default), not 0.25 — 0.25 makes 205-unknown pixels
  reload as free space (caught by the round-trip unit test).
- `NavExecutor.on_result` ignores results arriving after a terminal state — a late CANCELED
  from our own localization-loss cancel must not overwrite FAILED (caught by unit test).
- Scripted publishes to `/start_navigation` need `ros2 topic pub -w 1`; without waiting for
  subscription matching the one-shot message can be lost before DDS discovery completes
  (caught by the smoke test flaking between runs).
