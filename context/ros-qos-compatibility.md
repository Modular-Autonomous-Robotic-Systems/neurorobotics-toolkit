# ROS 2 QoS compatibility across `slam`, `controllers` and `sensors`

**Date:** 2026-07-25, revised the same day after the fixes landed. **Scope:** every topic edge shared between two or more packages in `/ws/ros_ws`, audited against its publisher. Change-record: [`/ws/plans/basalt-slam-defect-remediation.md`](../../plans/basalt-slam-defect-remediation.md).
**Publisher-side reference (SITL):** [`/ws/context/ardupilot-dds-qos.md`](../../context/ardupilot-dds-qos.md).

## The rule DDS actually applies

An endpoint pair matches only when the publisher's **offered** policy is at least as strong as the subscriber's **requested** policy:

- Reliability: `RELIABLE` ≥ `BEST_EFFORT`. A RELIABLE publisher satisfies a BEST_EFFORT request; a BEST_EFFORT publisher does **not** satisfy a RELIABLE request.
- Durability: `TRANSIENT_LOCAL` ≥ `VOLATILE`. A TRANSIENT_LOCAL publisher satisfies a VOLATILE request; a VOLATILE publisher does **not** satisfy a TRANSIENT_LOCAL request.

The counter-intuitive consequence, and the source of every QoS bug found in this workspace: **asking for more guarantees narrows the set of publishers you can talk to.** A mismatch produces no exception and no log line — the callback simply never fires.

## Audit (as of 2026-07-27)

| Topic | Publisher (offered) | Subscriber (requested) | Match? |
|---|---|---|---|
| `/ap/status` | AP_DDS — TRANSIENT_LOCAL, RELIABLE, KEEP_LAST(1) | `controllers/src/logging/video_controller.cpp:127-134` — depth 10 ⇒ VOLATILE, RELIABLE | ✅ |
| `/ap/status` | as above | `controllers/controllers/slam/driver_node.py:410-420` — VOLATILE, BEST_EFFORT, depth 1 | ✅ |
| `/ap/imu/experimental/data` | AP_DDS — VOLATILE, **BEST_EFFORT**, KEEP_LAST(5) | `slam/src/basalt/node.cpp` — `rclcpp::SensorDataQoS()` ⇒ VOLATILE, BEST_EFFORT, KEEP_LAST(5) | ✅ (was ❌ until 2026-07-25) |
| `/airsim_node/Copter/front_center_Scene/image` | ~~`airsim_ros_wrapper.cpp:265` — `image_transport::advertise(topic, 1)` ⇒ VOLATILE, RELIABLE, KEEP_LAST(1)~~ **superseded 2026-08-21**: `ros2 topic info -v` against a live Project AirSim `projectairsim_ros2_cpp_node` shows the offered QoS is actually VOLATILE, **BEST_EFFORT** (History reported UNKNOWN over the CLI). The RELIABLE claim above no longer matches observed behaviour — treat `image_transport::advertise(topic, 1)`'s default as not authoritative for Project AirSim's bridge. | `slam/src/basalt/node.cpp:108-110` — was depth 10 ⇒ VOLATILE, RELIABLE (❌, mismatched); now `rclcpp::SensorDataQoS()` ⇒ VOLATILE, BEST_EFFORT | was ❌, now ✅ (fixed 2026-08-21) |
| `/airsim_node/…/image` | as above | `slam/src/orbslam3/monocular.cpp:78-80` — was depth 10 ⇒ VOLATILE, RELIABLE (❌, mismatched); now `rclcpp::SensorDataQoS()` ⇒ VOLATILE, BEST_EFFORT | was ❌, now ✅ (fixed 2026-08-21) |
| `/airsim_node/…/image` | as above | `sensors` package `raw_to_ffmpeg_republisher` (`image_transport::republish`, launched from `sensors/launch/ffmpeg_encode.launch.py`) — requests RELIABLE by default | ❌ — **not fixed**; out of scope for the `slam` submodule, needs a `qos_overrides` reliability override in the `sensors` launch file if this republisher is to receive frames from Project AirSim |
| `/video_logging_controller_node/start_logging` | `ros2 topic pub` (launched from `controllers/launch/target_recorder.launch.py:123-134`) ⇒ VOLATILE, RELIABLE | `controllers/src/logging/video_controller.cpp:115-119` — depth 10 ⇒ VOLATILE, RELIABLE | ✅ |
| `/ap/status` | as above | `rosbag2_transport::Recorder` (inside `managed_logger_node`) — **negotiated at subscription time**, not fixed: `adapt_request_to_offers()` requests TRANSIENT_LOCAL + RELIABLE because the sole publisher offers both | ✅ |
| `/<ns>/tello_state` | `tello/tello_driver/src/tello_driver_node.cpp` — `rclcpp::QoS(1).transient_local()` ⇒ **TRANSIENT_LOCAL**, RELIABLE, KEEP_LAST(1); edge-driven, never republished (was VOLATILE until 2026-07-25) | `controllers/src/logging/video_controller.cpp:136-142` — depth 10 ⇒ VOLATILE, RELIABLE | ✅ |
| `/<ns>/tello_state` | as above | `controllers/controllers/slam/driver_node.py:421-425` — VOLATILE, BEST_EFFORT, depth 1 | ✅ |
| `/<ns>/tello_state` | as above | `controllers/src/tello/controller.cpp:47-51` — depth 10 ⇒ VOLATILE, RELIABLE | ✅ |
| `/<ns>/tello_state` | as above | `rosbag2_transport::Recorder` — negotiates **TRANSIENT_LOCAL** now that the sole publisher offers it | ✅ |
| `/{slam_compute_node_name}/transition_event` | `rcl_lifecycle`, created with the rcl publisher defaults ⇒ VOLATILE, RELIABLE, KEEP_LAST(10) | `controllers/controllers/slam/driver_node.py:380-386` — depth 10 ⇒ VOLATILE, RELIABLE (added 2026-07-27) | ✅ |
| `/video_reader/transition_event` | as above | `sensors/src/video/logger.cpp:291-294` — depth 10 ⇒ VOLATILE, RELIABLE | ✅ |

Every row now matches. The one that did not, until 2026-07-25, was the Basalt VISLAM IMU subscription: `create_subscription<sensor_msgs::msg::Imu>(topic, 10, cb)` requests RELIABLE, ArduPilot offers BEST_EFFORT, the endpoints never connected — so the VIO estimator received zero IMU measurements while every log line and `ros2 node info` output looked healthy. It now requests `rclcpp::SensorDataQoS()` — BEST_EFFORT, VOLATILE, KEEP_LAST(5) — an exact match for the offer. **This is the canonical example of the rule below: the bug was asking for *more* than the publisher offered.**

## The two rules this workspace follows

1. **Never request `TRANSIENT_LOCAL` on a subscriber.** A TRANSIENT_LOCAL subscriber does not match a VOLATILE publisher, and `ros2 bag play` (`controllers/launch/replay_active_mono.launch.py:63-69`) republishes VOLATILE unless explicitly overridden. Requesting durability buys a latched first sample at the cost of breaking every replay-driven workflow — not a trade this stack wants. This is why the `/ap/status` subscribers stay VOLATILE even though the publisher offers TRANSIENT_LOCAL: they forgo the latched sample and pick up the next periodic one instead.
1a. **The dual of rule 1: *offering* `TRANSIENT_LOCAL` on a publisher is always safe, and is the right fix for an edge-driven state topic.** Offering more never breaks an existing subscriber, because matching requires offered ≥ requested — so promoting `tello_state` from VOLATILE to TRANSIENT_LOCAL left `video_controller.cpp`, `driver_node.py` and `tello/controller.cpp` untouched and still matching. Reach for this whenever a topic publishes only on state changes and has no keep-alive timer, since otherwise its value is unobservable to anything that joins between edges. Do **not** reach for it on high-rate streams, where retaining samples costs memory for no benefit.
2. **Request `BEST_EFFORT` for sensor streams; request `RELIABLE` only where the publisher already offers it.** BEST_EFFORT is the weakest request and therefore matches the widest set of publishers, live and replayed. Use `rclcpp::SensorDataQoS()` for imagery and IMU; a bare depth (which means RELIABLE) is acceptable only for state and command topics whose publisher is known-RELIABLE.

**Rule 1 applies to hand-written subscribers, not to the bag recorder.** `rosbag2_transport::Recorder` negotiates per topic against the live publisher set (`adapt_request_to_offers()`), so it *does* request TRANSIENT_LOCAL on `/ap/status` — and that is desirable, because it means the recorder starts with a status sample rather than waiting for the next one. (It is **not** sufficient to capture the arming trigger: depth is 1, so by activation time that sample has been superseded by the flying status. See the next section.) The recorder can do this safely precisely because it decides at subscription time against the publishers actually present, which a statically written subscriber cannot. Never impose a blanket `topic_qos_profile_overrides` on the recorder: it bypasses that negotiation. See [`rosbag2-recording-architecture.md`](rosbag2-recording-architecture.md).

## Matching is not the same as capturing

Every `/<ns>/tello_state` row above matched even when the recorder captured **no** trigger message on the Tello path. Matching governs whether messages published *while both endpoints exist* are delivered. Capturing a message published *before* the subscriber existed additionally requires the publisher to have retained it, i.e. `TRANSIENT_LOCAL` durability. `/ap/status` always had it (depth 1); `tello_state` did not until 2026-07-25. When auditing "why is this message missing from the bag", check retention before checking compatibility — a green match column proves nothing about the first message. See [`rosbag2-recording-architecture.md`](rosbag2-recording-architecture.md).

Retention alone is still not enough on a depth-1 latch: the sample must also not have been *superseded* before the recorder subscribes. That is why the ArduPilot fix was a lifecycle change (activate on the arming edge, not the later flying edge) rather than a QoS change — see [`slam-lifecycle-flow.md`](slam-lifecycle-flow.md).

## Why `/ap/status` needs no change, in either back-end

`controllers/controllers/slam/driver_node.py` is the **shared** base class for both `orbslam3_monocular_controller` and `basalt_slam_controller` (`mono_driver_node.py` and `basalt_driver_node.py` both derive `SLAMDriverNode`), so its `/ap/status` QoS is the one setting that genuinely spans the two SLAM back-ends. It requests BEST_EFFORT + VOLATILE, which is the weakest possible request and matches AP_DDS, bag replay and any hand-rolled test publisher alike. It is already correct; changing it in either direction would narrow compatibility. The C++ `video_controller.cpp` requests RELIABLE + VOLATILE, which also matches because AP_DDS offers RELIABLE — but note it is the more fragile of the two: it would stop working if ArduPilot ever downgraded `/ap/status` to BEST_EFFORT, whereas the Python side would not.

The IMU topic has **no ORB-SLAM3 counterpart** — `slam/src/orbslam3/monocular.cpp` subscribes to the camera only (`:78`) and publishes `~/annotated_frame` and `~/map_points` (`:82-83`). So the Change 2.4 fix propagates nowhere: there is nothing on the ORB-SLAM3 side to keep in step.

## Diagnosing a suspected mismatch

```bash
ros2 topic info -v /ap/imu/experimental/data   # prints offered and requested QoS per endpoint
ros2 node info /basalt_slam_node               # proves the subscription EXISTS, not that it MATCHED
ros2 topic hz /ap/imu/experimental/data        # data actually flowing?
```

`ros2 topic info -v` is the only one of the three that distinguishes a healthy system from a QoS-incompatible one. `ros2 node info` looks identical in both cases, which is exactly why this class of bug survives code review.

See also [`slam-lifecycle-flow.md`](slam-lifecycle-flow.md) for the lifecycle/service side of the same startup path, [`launch-and-djinn.md`](launch-and-djinn.md) for how these topic names are threaded through the launch graph, and [`controllers-package.md`](controllers-package.md) for the driver class hierarchy.
