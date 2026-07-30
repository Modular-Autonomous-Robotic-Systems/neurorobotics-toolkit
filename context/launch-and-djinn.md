# Launch graph, arg propagation & djinn

**Revised 2026-07-26**: `basalt_slam_test.launch.py` no longer includes `sitl.launch.py`. Supersedes the 2026-07-25 revision that documented argument forwarding between the two ([`/ws/plans/basalt-slam-defect-remediation.md`](../../plans/basalt-slam-defect-remediation.md) Change 3.4).

## Two independent stacks

The Basalt SITL setup is brought up by **two separate `djinn` invocations**, and neither launch file includes the other.

```
djinn start sitl                     (persistent, spawns airsim_container)
controllers/launch/sitl.launch.py
  └── TimerAction(3s): airsim_ros_pkgs, ardupilot_sitl (sitl_dds_udp),
      sensors/ffmpeg_encode, controllers/target_recorder.launch.py
           ├── includes sensors/launch/managed_logger.launch.py (managed_logger_node)
           └── controllers/video_logging_controller_node

djinn start sitl basalt              (execs into the running nrt_container)
controllers/launch/basalt_slam_test.launch.py
  ├── includes slam/launch/basalt_slam.launch.py  → basalt_slam_node (compute)
  └── TimerAction(3s): controllers/basalt_slam_controller (the Python driver)
```

**`basalt_slam_test.launch.py` is SLAM-only and owns no simulator or recorder actions.** It included `sitl.launch.py` until 2026-07-26, which meant `djinn start sitl basalt` stood up a *second* AirSim bridge, ArduPilot SITL, ffmpeg encoder and `managed_logger_node` on top of the ones `djinn start sitl` had already started. Two `managed_logger_node` instances race for the same bag path, and the loser's `on_activate` throws `Database directory already exists (/ws/data/telemetry)` — see [`rosbag2-recording-architecture.md`](rosbag2-recording-architecture.md). If you ever need SLAM and the simulator from one file, add a new composed launch file rather than re-adding the include here.

`ffmpeg_encode` is included by `sitl.launch.py` only. `target_recorder.launch.py` used to carry a commented-out duplicate include of it that referenced an undefined `delayed_launch`; that dead block was deleted 2026-07-25 so it can never be uncommented into a double launch.

## Arg propagation — the rule, and the trap

CLI `key:=value` sets a **global** `LaunchConfiguration`. An included file's `DeclareLaunchArgument key` only supplies a default when the key is not already set, so CLI values *do* reach same-named arguments in includes even without an explicit `launch_arguments=`.

**The trap: an argument that no launch file in the chain declares is silently discarded.** It becomes a top-level configuration that nothing consumes — no error, no warning. This is how `ap_type`, `output_bag_name` and `topics_to_record` from `/ws/djinn` once reached nothing at all: `basalt_slam_test.launch.py` declared none of them and passed no `launch_arguments` to `sitl.launch.py`.

**The second trap: the key name must match the *declared* name exactly, hyphens included.** `sitl.launch.py` forwarded `"log_level"` while `target_recorder.launch.py` declares `"log-level"`, so `log-level:=DEBUG` never reached the video logging controller and it silently kept its `INFO` default. Both spellings exist in this workspace — `log-level` (hyphen) in the launch files, `log_level` nowhere — so grep the declaration before writing a forwarding dict.

**Current state (2026-07-26).** The forwarding chain between `basalt_slam_test.launch.py` and `sitl.launch.py` no longer exists, because the include was removed. Ownership is now clean:

| Launch file | Declares | Forwards to |
|---|---|---|
| `basalt_slam_test.launch.py` | `log-level`, `camera-topic`, `imu-topic`, `calibration-file-path`, `configuration-file-path`, `slam-type`, `drone-type`, `ap_status_topic_name`, `use-sim-time` | `basalt_slam.launch.py` (SLAM args only, plus `use-sim-time`); `ap_status_topic_name` goes to `basalt_slam_controller` as the `ardupilot_status_topic` param |
| `sitl.launch.py` | `log-level`, `ap_status_topic_name`, `ap_type`, `output_bag_name`, `topics_to_record`, `use-sim-time` | all six into `target_recorder.launch.py` under the correct (hyphenated) keys |

**`use-sim-time` (added 2026-07-28) is the third instance of the hyphen trap, and it was hit three times in one change.** The launch argument is `use-sim-time`; the *node parameter* it sets and the `ardupilot_sitl` launch argument that becomes `--use_sim_time` are both spelled `use_sim_time`. Three forwarding dicts were first written with the underscored key — `sitl.launch.py` → `target_recorder.launch.py`, `target_recorder.launch.py` → `managed_logger.launch.py`, and `basalt_slam_test.launch.py` → `basalt_slam.launch.py` — and all three silently set a launch configuration nothing reads, leaving the recorder and the SLAM compute node on wall clock while the rest of the stack ran on simulator time. Every forwarding site now carries a comment naming the spelling. The one place the underscored key is correct is `sitl.launch.py`'s `ardupilot_sitl` dict, because that is the name upstream declares (`Tools/ros2/ardupilot_sitl/src/ardupilot_sitl/launch.py:663-668`).

Each declaration carries `choices=["True", "False"]`, which is deliberately stricter than the implicit `yaml.safe_load` type inference `launch_ros` applies to a bare `LaunchConfiguration` in a parameter dict — it rejects `use-sim-time:=yes` when the argument is declared rather than when a parameter is read, and it covers the forwarding sites, which a `ParameterValue(value_type=bool)` wrapper cannot. The set matches `BOOL_STRING_CHOICES` in the `ardupilot_sitl` launch file, so any value accepted here is accepted downstream. See [`/ws/context/sitl-time-synchronisation.md`](../../context/sitl-time-synchronisation.md).

`sitl.launch.py` is the sole owner of the recorder. Recorder settings are not reachable from `basalt_slam_test.launch.py`'s command line by design — pass them to `djinn start sitl` / `sitl.launch.py`.

**`topics_to_record` exists in four places** — `sitl.launch.py`, `target_recorder.launch.py`, `sensors/launch/managed_logger.launch.py` and `/ws/djinn`'s `start sitl` path — and they are **not** all identical (the `sitl.launch.py` copy includes `/front_center_camera/compressed`; `target_recorder.launch.py`'s does not, and has `/tf`/`/tf_static` instead). The `basalt_slam_test.launch.py` copy was the fifth and was deleted 2026-07-26. Collapsing the remaining four to one source of truth is a known follow-up.

## slam/launch/basalt_slam.launch.py — args → params

Args: `log-level`, `camera-topic` (`/camera`), `calibration-file-path` (`""`), `configuration-file-path` (`""`), `slam-type` (`VSLAM`), `imu-topic` (`""`), `use-sim-time` (`False`). Node `basalt_slam_node` params: `camera_topic_name`, `calibration_file_path`, `configuration_file_path`, `slam_type`, `imu_topic_name`, `use_sim_time`. The node fails configuration loudly if `slam_type == VISLAM` and `imu_topic_name` is empty, rather than subscribing to `""`.

**Corrected 2026-07-27:** the `gdb` prefix is now commented out (`basalt_slam.launch.py:73`), so the node runs unwrapped. The earlier claim that it is "kept unconditional by decision" is stale. `/ws/djinn` still uses `kill -9` to tear the stack down, which was originally needed because a gdb-wrapped node does not forward `SIGTERM` normally.

## slam_controller.launch.py (ORB_SLAM3 reference for controller wiring)

TimerAction(3s): `slam/orbslam3_mono_node` (compute) + `controllers/orbslam3_monocular_controller` (the `MonoDriver`) with params `settings_file_path`, `camera_topic_name`, `vocab_file_path`, `drone_type`, `ardupilot_status_topic`, `tello_status_topic`.

## djinn — `/ws/djinn` (bash dispatcher, NRT_WS=/ws)

`djinn start sitl` (no third argument) spawns the persistent `airsim_container` via `start_simulation` in [`/ws/scripts/helpers/sitl.sh`](../../scripts/helpers/sitl.sh). `sitl.launch.py` itself is launched by [`/ws/ardu_ws/scripts/run_ardupilot_airsim_ros_sitl.sh`](../../ardu_ws/scripts/run_ardupilot_airsim_ros_sitl.sh) into `sitl_bridge_container`.

`djinn start sitl basalt` then runs, inside `nrt_container` with `DISPLAY=:1`:

```
ros2 launch controllers basalt_slam_test.launch.py \
  log-level:=DEBUG ap_status_topic_name:=/ap/status \
  camera-topic:=/airsim_node/Copter/front_center_Scene/image \
  calibration-file-path:=/ws/ros_ws/src/slam/ext/basalt/data/sitl_calib.json \
  configuration-file-path:=/ws/ros_ws/src/slam/ext/basalt/data/sitl_config_vo.json \
  slam-type:=VISLAM use-sim-time:=True imu-topic:=/ap/imu/experimental/data
```

`use-sim-time:=True` (2026-07-28) reaches `basalt_slam_node` only. It must be paired with the same argument on `djinn start sitl`, which is what actually publishes `/clock`; a sim-time node with no `/clock` publisher reports time zero forever. Note that there is no space after `:=` — `ros2 launch` splits on whitespace before splitting on `:=`, so `use-sim-time:= True` is two tokens and fails the launch.

then kills leftover `basalt` processes with `kill -9`.

- Every argument in that list is declared by `basalt_slam_test.launch.py`, so none is silently discarded. Keep it that way: adding an argument here without a matching `DeclareLaunchArgument` is a no-op.
- **Removed from this invocation 2026-07-26**, when the `sitl.launch.py` include was deleted: `input_topic`, `ffmpeg_topic`, `ffmpeg_node_name`, `ap_type`, `output_bag_name`, `topics_to_record`. The first three were *always* discarded (no launch file in the chain ever declared them; `sitl.launch.py` hardcodes the encoder's topics). The last three became recorder-only settings that now belong to `djinn start sitl`.
- `ap_type` was `ArduPilot` until 2026-07-25 and matched nothing — `video_controller.cpp` compares against lowercase `"ardupilot"` exactly. Now lowercase, and an unrecognised value produces an explicit warning instead of a silent no-subscriber.
- `sitl_config_vo.json` is a full **VIO** config despite the `_vo` filename — the suffix is a naming artefact, not a mode selector. `slam-type:=VISLAM` is the authority.
- `djinn start sitl orbslam3` runs `ros2 launch slam orbslam3.launch.py`, which **does not exist** (`slam/launch/` contains only `basalt_slam.launch.py`). That workflow cannot work; see [`/ws/context/djinn-workflows.md`](../../context/djinn-workflows.md).
