# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

**Justice** is an autonomous fixed-wing UAV pursuit system built for the **TEKNOFEST Savaşan İHA** competition. It implements a **hybrid GPS + vision** guidance pipeline:

1. Long range — **L1 Adaptive Guidance** flies the aircraft toward a target's GPS coordinate.
2. Close range — **YOLO + TCTrack++** vision tracker locks onto the target in the camera frame; a **PID servo** loop converts pixel error into velocity/yaw-rate setpoints.
3. After 4 s of continuous visual lock, the FSM declares `KİLİTLENME` (lock confirmed).

Stack: **ROS 2 Humble**, **PX4 Autopilot v1.14+** (Offboard mode via micro-XRCE-DDS), **Gazebo Harmonic**, Python 3.10+, PyTorch (CUDA optional).

## Common Commands

There is **no test suite** in this repo. `package.xml` declares `ament_flake8` / `ament_pep257` test dependencies but no tests are wired up. Linting is not part of the regular workflow.

### Build
```bash
# px4_msgs must be available first (separate workspace)
cd ~/ros2_ws/src && git clone https://github.com/PX4/px4_msgs.git -b release/1.14
cd ~/ros2_ws && colcon build --packages-select px4_msgs
source ~/ros2_ws/install/setup.bash

# then build justice
source /opt/ros/humble/setup.bash
colcon build --packages-select justice
source install/setup.bash
```

### Launch the system
```bash
ros2 launch justice justice_sim.launch.py
# Optional args: px4_ns:=/px4_1 base_speed:=15.0
```

### Run a single node directly
```bash
ros2 run justice justice_node
ros2 run justice tracker_node
ros2 run justice mock_target_receiver
```

### SITL bring-up (full sim — 4 terminals)
```bash
# Terminal 1 — Tracker (Instance 0, with camera)
cd ~/PX4-Autopilot
PX4_GZ_WORLD=default PX4_SYS_AUTOSTART=4007 PX4_GZ_MODEL_POSE="0,0,0,0,0,0" \
PX4_INSTANCE=0 PX4_UXRCE_DDS_PORT=8888 make px4_sitl gz_rc_cessna_mono_cam

# Terminal 2 — Target (Instance 1, no camera)
cd ~/PX4-Autopilot
PX4_GZ_WORLD=default PX4_SYS_AUTOSTART=4003 PX4_GZ_MODEL_POSE="10,0,0,0,0,0" \
PX4_INSTANCE=1 PX4_UXRCE_DDS_PORT=8889 ./build/px4_sitl_default/bin/px4 -i 1

# Terminal 3, 4 — DDS Agents
MicroXRCEAgent udp4 -p 8888
MicroXRCEAgent udp4 -p 8889
```

Model weights (~134 MB total) are **not** in git. Drop `best.pt` (YOLO) and `TCTrack.pth` (TCTrack++) into `justice/tracking/snapshot/` or `tracker_node` will fail at startup.

## Architecture

### Nodes (all started by `justice_sim.launch.py`)

| Node | File | Role |
|------|------|------|
| `gz_camera_bridge` | (ros_gz_image plugin) | Bridges Gazebo `/world/.../camera/image` → `/sensor/camera/image_raw` |
| `tracker_node` | `justice/tracking/tracker_node.py` | YOLO search → TCTrack++ pursuit → publishes pixel error + bbox area |
| `justice_node` | `justice/justice_node.py` | Main FSM; consumes vision + PX4 telemetry + competition targets; publishes `TrajectorySetpoint` |
| `mock_target_receiver` | `justice/mock_target_receiver.py` | Test helper: relays a WebSocket-based competition feed → `/competition/target_list` |

A **MultiThreadedExecutor (4 threads)** runs `justice_node`. The single `_kontrol_dongusu` timer (50 Hz, configurable via `px4.control_frequency`) owns all state-machine writes; other callbacks only update shared snapshot variables.

### Topic graph (the ones to memorize)

**`justice_node` subscribes:**
| Topic | Type | QoS | Purpose |
|-------|------|-----|---------|
| `/fmu/out/vehicle_local_position` (+`_v1`) | `VehicleLocalPosition` | BEST_EFFORT | Own NED position/velocity |
| `/fmu/out/vehicle_global_position` | `VehicleGlobalPosition` | BEST_EFFORT | Own GPS |
| `/fmu/out/vehicle_attitude` | `VehicleAttitude` | BEST_EFFORT | Quaternion → Euler |
| `/fmu/out/vehicle_status` (+`_v1`) | `VehicleStatus` | BEST_EFFORT (+TRANSIENT_LOCAL) | ARM / `nav_state` |
| `/vision/target_state` | `geometry_msgs/PointStamped` | RELIABLE | `x=pixel error X, y=pixel error Y, z=bbox area` |
| `/competition/target_list` | `std_msgs/Float64MultiArray` | RELIABLE | Flattened `[id, lat, lon, alt, speed, heading]×N` |
| `/simulation/enemy_aircraft` | `std_msgs/String` (JSON) | RELIABLE | Legacy sim target feed |
| `{px4_ns}/fmu/out/vehicle_(local\|global)_position` | (same) | BEST_EFFORT | **Target** aircraft telemetry (Instances 1+) |

**`justice_node` publishes:**
| Topic | Type | Purpose |
|-------|------|---------|
| `/fmu/in/trajectory_setpoint` | `TrajectorySetpoint` | Position (GPS_TAKİP) or velocity+yawspeed (GÖRSEL/KİLİTLENME) |
| `/fmu/in/offboard_control_mode` | `OffboardControlMode` | **100 Hz heartbeat** — drops offboard after ~0.5 s without it |
| `/fmu/in/vehicle_command` | `VehicleCommand` | Mode change / arm |
| `/justice/state` | `String` | Current FSM state name (UI/monitoring) |
| `/justice/virtual_target` | `PoseStamped` | L1 virtual waypoint, **converted to ENU** for RViz |
| `/justice/target_scores` | `String` (JSON) | Target selector scores |
| `/justice/debug_info` | `String` | Free-form debug |

**`tracker_node`:** subscribes `/sensor/camera/image_raw` (sensor QoS), publishes `/vision/target_state` and `/tracker/debug_image`.

Two PX4 vehicles run in SITL: Instance 0 (`/fmu/...`, tracker, sys_id=1) and Instance 1 (`/px4_1/fmu/...`, target, sys_id=2). The `namespaces.targets` param (`/px4_1,/px4_2`) tells `justice_node` which target namespaces to subscribe to.

## FSM (`class Durum` in `justice_node.py`)

| State | Control output | Trigger to enter |
|-------|----------------|------------------|
| `BEKLEME` (0) | — | Initial; waits for first `VehicleStatus` |
| `KALKIS_BEKLE` (1) | — | `px4_connected == True` |
| `OFFBOARD_GECIS` (2) | OffboardControlMode heartbeats | `px4_armed == True` |
| `GPS_TAKIP` (3) | NED **position** setpoint (L1) | 150 heartbeats sent **AND** `nav_state == NAVIGATION_STATE_OFFBOARD (14)`; bypass after 20 retries (~30 s) |
| `GORSEL_TAKIP` (4) | NED **velocity + yawspeed** (PID) | `target_area>0` AND `gps_target_distance < transition_distance` AND `consecutive_detections ≥ transition_min_detections` |
| `KILITLENME` (5) | Same as `GORSEL_TAKIP` | 4 s continuous `target_area > area_ref*0.3` |

**Fallbacks / safety:**
- `GORSEL_TAKIP` / `KILITLENME` → `GPS_TAKIP` if `time_since_vision > visual_lost_timeout` (10 s).
- PX4 connection watchdog: 3 s without `VehicleStatus` flips `px4_connected=False`.
- `_publish_safe_loiter()` emits a cruise-airspeed velocity heading-hold when no valid target exists.
- Stale targets older than 5 s are silently dropped (`_cleanup_stale_targets`).
- The 100 Hz `OffboardControlMode` heartbeat runs on its own timer — **do not change its rate**; PX4 will exit Offboard if it stops.
- VehicleStatus is subscribed on **both** `/fmu/out/vehicle_status` and `..._v1` because PX4 message namespaces differ between SITL builds — keep both.

**Parameter precedence:** `justice_sim.launch.py` sets parameters inline (e.g. `transition.gps_to_visual_distance=200.0`, `min_consecutive_detections=5`) — these **override** the defaults in `config/justice_params.yaml` (30.0 / 3). When tuning, edit the launch file or pass `--ros-args`; YAML values only apply if no override is present.

## L1 Guidance (`justice/l1_guidance.py`)

Public entry point: `L1Guidance.update(own_state: AircraftState, targets: List[Target], current_time: float) → GuidanceCommand`.

**Core math** (in `L1GuidanceController.compute`, ~line 1093):
```
virtual_target = target_pos − l1_distance * (target_vel / ‖target_vel‖)   # lead pursuit
η              = angle(LOS, current_heading_vector)
l1_accel       = 2 * v² / l1_distance * sin(η)
bank_cmd       = atan(l1_accel / g)
crab_angle     = correction from cross-track error
```

**Adaptive L1 distance** (when `adaptive_l1: true`):
```
l1_distance *= (1 + v_target / 30.0)
l1_distance  = clip(l1_distance, 30.0, 100.0)   # metres
```

**Internal `TrackingStateMachine`** (sub-FSM for L1, distance-driven):
`IDLE → SEARCHING → APPROACHING (<approach_distance 50 m) → PURSUING (<lock_distance 15 m) → LOCKED (<80% lock_distance) ↔ LOITERING (<loiter_trigger_distance 8 m, tail-chase)`.

**`WeightedTargetSelector`** (multi-target scoring; weights must sum ≈ 1.0):
- `distance` 0.25 — closer is better (sigmoid)
- `angle` 0.30 — tail-cone bonus ×2.5 (aspect < 45°), head-on penalty ×0.2 (aspect > 150°)
- `speed` 0.15 — prefer slower / catchable targets
- `camera_fov` 0.20 — bonus ×1.5 if target inside FOV cone
- `reachability` 0.10 — intercept-time feasibility (max 60 s, prefer <30 s)

Output of `L1Guidance.update` is a `GuidanceCommand` containing an **NED position setpoint** — PX4's TECS / inner loop handles bank, throttle, pitch. `justice_node` writes this position directly into `TrajectorySetpoint.position` during `GPS_TAKIP`. The `CommandSmoother` applies rate limits (`heading_rate_limit`, `altitude_rate_limit`) and a low-pass filter (`lpf_cutoff_heading`, `command_smoothing_alpha`) before output.

## Visual Servo (`GORSEL_TAKIP` / `KILITLENME`)

Driven by `/vision/target_state` (`x = pixel error X`, `y = pixel error Y`, `z = bbox area`). PID gains: yaw `0.002 / 0.0001 / 0.001`, pitch `0.003 / 0.0001 / 0.001`. Limits: `max_yaw_rate = 0.5 rad/s`, `max_vz = 3 m/s`. Forward speed is modulated by `(area_ref − current_area) * kp_speed`, clipped to `[min_speed, max_speed]`.

**Body→NED rotation** (yaw `ψ` is current aircraft heading):
```
vN = vx_body · cos(ψ) − vy_body · sin(ψ)
vE = vx_body · sin(ψ) + vy_body · cos(ψ)
vD = −pitch_pid
```
PID integrators reset on every fallback from visual to GPS mode.

## Vision Tracker (`tracker_node.py`)

Two-stage pipeline:
1. **Search (YOLO `best.pt`)** — runs on every frame until a detection with confidence > 0.3, bbox ≥ 1% of image, and inside the "yellow boundary" (10–90% horizontal, 5–95% vertical) is found.
2. **Track (TCTrack++)** — `pysot.ModelBuilder_tctrackplus` + `TCTrackplusTracker`, initialized from the YOLO ROI. YOLO is then **disabled** until tracking is lost.

**Lock state machine inside the tracker:**
- Match score ≥ `KAYIP_ESIGI` (0.80) → "good"; ROI must be in boundary box AND ≥ 2% of image.
- 4 s continuous "good" → publish "LOCKED SUCCESSFULLY".
- Score < 0.80 → tolerance counter; if > `TOLERANS_SURESI` (1.0 s) → exit tracking.
- Darkness guard: ROI mean grayscale < 10 for 5 consecutive frames → forced exit to search.

The tracker publishes `point.z = bbox area` as the proxy for "target distance" — `justice_node` uses this directly to compare against `area_ref`. There is no explicit pose / 3D target estimate from vision.

## Parameter Guide (`config/justice_params.yaml`)

Loaded under namespace `justice_node.ros__parameters`. Parameters are dot-hierarchical (`l1_guidance.l1_distance`). **Note:** `l1_damping`, `l1_period`, and `loiter_radius` are declared but **not used in the current math** — do not assume tuning them does anything.

| Group | Key params (default) | Effect |
|-------|----------------------|--------|
| `l1_guidance` | `l1_distance` 50 m, `adaptive_l1` true | Lead distance; larger ⇒ smoother turns, more lag |
| `weights` | distance 0.25, angle 0.30, speed 0.15, camera_fov 0.20, reachability 0.10 | Target selector weighting (sum ≈ 1.0) |
| `angle_scoring` | `tail_angle_bonus` 2.5, `head_on_penalty` 0.2, `tail_cone_half_angle` 45°, `head_on_cone_half_angle` 30° | Aspect-angle scoring |
| `aircraft_limits` | `min_airspeed` 15, `cruise_airspeed` 22, `max_airspeed` 35 m/s, `max_bank_angle` 45° | Flight envelope; bank used by L1 |
| `smoothing` | `heading_rate_limit` 15°/s, `altitude_rate_limit` 3 m/s, `command_smoothing_alpha` 0.15, `lpf_cutoff_heading` 0.5 Hz | Output filtering in `CommandSmoother` |
| `state_machine` | `lock_distance` 15, `approach_distance` 50, `loiter_trigger_distance` 8 m, `lock_confirmation_time` 4 s | L1 sub-FSM thresholds |
| `visual` | yaw PID 0.002/0.0001/0.001, pitch PID 0.003/0.0001/0.001, `max_yaw_rate` 0.5 rad/s, `max_vz` 3 m/s, `base_speed` 15, `area_ref` 5000 px², `kp_speed` 0.001, `min_speed` 10, `max_speed` 25 m/s | Visual-servo loop |
| `transition` | `gps_to_visual_distance` 30 (YAML) / **200 (launch override)** m, `min_consecutive_detections` 3 / **5**, `visual_lost_timeout` 10 s | GPS↔Visual gating |
| `px4` | `control_frequency` 50 Hz, `namespace` `''`, `mav_sys_id` 1 | Main loop rate, PX4 wiring |
| `safety` | `min_altitude_agl` 50 m | Geofence floor (advisory — no hard enforcement in code) |
| `namespaces.targets` | `'/px4_1,/px4_2'` | Comma-separated target vehicle namespaces |

## Coordinate frames & units

- **NED (North-East-Down) everywhere internally.** All position/velocity setpoints sent to PX4 are NED.
- **GPS ↔ NED** via `geodetic_to_ned()` / `ned_to_geodetic()` with a spherical Earth (R = 6 371 000 m). Reference is the takeoff "home" position; **home must be set** before any L1 calculation.
- **ENU is only used for `/justice/virtual_target`** (RViz convention — `z` is negated when publishing the `PoseStamped`).
- **Angles:** heading / bearing / aspect angle in **degrees** (0–360, North=0, East=90). Yaw setpoints inside `GuidanceCommand` and on `TrajectorySetpoint.yawspeed` are in **radians**. Helpers: `normalize_angle` (±180°), `normalize_angle_rad` (±π), `wrap_to_360`, `wrap_to_2pi`.
- **Distances/positions:** metres. **Speeds:** m/s.
- **PX4 constants:** `NAVIGATION_STATE_OFFBOARD = 14`, `ARMING_STATE_ARMED = 2`.

## Code conventions (observed)

- Classes `PascalCase` (`JusticeNode`, `L1Guidance`, `WeightedTargetSelector`, `TrackingStateMachine`, `CommandSmoother`, `PIDController`).
- Private methods `_snake_case` (`_durum_bekleme`, `_kontrol_dongusu`, `_publish_position_setpoint`, `_change_durum`).
- Constants `UPPER_SNAKE_CASE`.
- **FSM state identifiers are Turkish** (`Durum.BEKLEME`, `KALKIS_BEKLE`, `OFFBOARD_GECIS`, `GPS_TAKIP`, `GORSEL_TAKIP`, `KILITLENME`). The same strings are published on `/justice/state` — do **not** anglicize them.
- ROS parameter names use dot hierarchy (`l1_guidance.l1_distance`, `visual.kp_yaw`).
- Topics under `/justice/...` are project-owned debug/state; `/fmu/...` is PX4-owned; `/vision/...`, `/competition/...`, `/simulation/...` are subsystem boundaries.

## Gotchas

- `px4_msgs` must be built in a sibling workspace **before** building `justice`; the package will not find headers otherwise.
- Snapshot weights (`best.pt`, `TCTrack.pth`) are `.gitignore`d and ~134 MB combined — `tracker_node` raises `FileNotFoundError` at construction if either is missing.
- `OffboardControlMode` heartbeat is **100 Hz on a dedicated timer** — PX4 leaves Offboard within ~0.5 s if the rate drops.
- `VehicleStatus` is subscribed on both `/fmu/out/vehicle_status` and `/fmu/out/vehicle_status_v1` to support different PX4 message builds — keep both subscriptions.
- `/competition/target_list` is a positional `Float64MultiArray` (`[id, lat, lon, alt, speed, heading]×N`) — there is no MultiArrayLayout-based labelling.
- Launch-file parameter overrides supersede `justice_params.yaml`. Several YAML/launch values disagree (e.g. `gps_to_visual_distance` 30 vs 200) — runtime uses the launch value.
- Several declared parameters (`l1_damping`, `l1_period`, `aircraft_limits.loiter_radius`) are not consumed by the current L1 math; tuning them has no effect.
