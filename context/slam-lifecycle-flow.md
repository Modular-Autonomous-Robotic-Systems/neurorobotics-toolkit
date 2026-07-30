# SLAM and recorder lifecycle control flow

Both lifecycle managers in this workspace, the Python SLAM driver and the C++ video logging driver, watch the same autopilot status topics and drive one managed node each. Since 2026-07-27 they are written on one common shape, the target-state reconciler, described in its own section below. Read that section first, then the per-manager sections.

## Actors

- Compute node (C++ `rclcpp_lifecycle::LifecycleNode`), `orbslam3_mono_node` or `basalt_slam_node`. Exposes the standard lifecycle services `/{node}/change_state` and `/{node}/get_state`, and publishes `lifecycle_msgs/msg/TransitionEvent` on `/{node}/transition_event`.
- Recorder node (C++ `rclcpp_lifecycle::LifecycleNode`), `managed_logger_node`, driven by `VideoLoggingDriver`.
- Driver/controller nodes, `SLAMDriverNode` (Python `rclpy.node.Node`, base of `MonoDriver` and `BasaltSLAMDriver`) and `VideoLoggingDriver` (C++, derives `LifecycleControllerBase`). Neither is itself a lifecycle node, both are managers that call the managed node's lifecycle services based on autopilot status.

## The target-state reconciler pattern (IMPLEMENTED 2026-07-27)

The pattern separates the policy question from the mechanism question, so that "where should the managed node be" and "which transition gets it there" are never interleaved.

1. A target state is derived from the vehicle phase alone, `armed` and `flying`, with no lifecycle predicates in it.
2. A step table maps `(knownState, targetState)` to the single transition that moves one step. The only `CLEANUP` row is `(INACTIVE, UNCONFIGURED)`, which is what structurally guarantees that a cleanup is never dispatched from `ACTIVE` and that the deactivate step is never skipped.
3. A reconciler reads the state mirror, looks up the step, claims an in-flight guard and dispatches asynchronously. Chains are never written out. The response handler updates the mirror and re-enters the reconciler, which takes the next step. The longest chain is two hops, so re-entry terminates.

Three rules make it correct, each learned from a defect in this tree.

The whole decision, that is read the mirror, compute the target, look up the step and claim the guard, happens inside one critical section. Splitting it lets two concurrent callbacks compute the same step and dispatch it twice. The service call is dispatched after the lock is released, because nothing past the claim reads guarded state, and holding a lock across a service call serialises the callback groups behind it for no benefit.

A failed or rejected transition re-syncs the mirror but does not re-enter the reconciler. Acting on the correction immediately retries the failed transition with no rate limit, which turns a persistent fault, for example `slam_type=VISLAM` with an empty `imu_topic_name`, into a service call storm. The next status message drives the retry instead, at the autopilot's 2 Hz keep-alive rate. In C++ this is enforced by testing the `get_state` context string against `"after_failed_change_state"` (`video_controller.cpp:509-513`), in Python by the `reconcile_after` argument of `_query_state` and by `RECONCILE_AFTER_TRANSITIONS`, which lists only the five success outcomes.

The two managers differ in one deliberate respect. The SLAM driver's target is recomputed from scratch on every message, because its specification is phrased as a level condition. The video logger's target is latched in `mpTargetState`, because its policy is genuinely edge-sensitive, the arming edge fires once and its effect must persist afterwards. A stateless target function would not reproduce the existing recording behaviour.

Full design and the case-by-case equivalence argument for the C++ side in [`/ws/plans/slam-lifecycle-ap-status-reconciler.md`](../../plans/slam-lifecycle-ap-status-reconciler.md) §4 and §6.

## SLAM driver flow (`controllers/controllers/slam/driver_node.py`)

1. The constructor declares parameters, creates the `ChangeState` and `GetState` clients on `/{slam_compute_node_name}/...` in a `ReentrantCallbackGroup`, subscribes to `/{slam_compute_node_name}/transition_event` in that same group, creates the status subscription in the node's default group, and creates one repeating timer. It performs no blocking call, see the workspace rule below.
2. `_await_lifecycle_services` (`:431`) polls both clients with the non-blocking `service_is_ready()` once a second, cancels its own timer when both are up, and issues the seeding `get_state`.
3. The status subscription is BEST_EFFORT, KEEP_LAST, depth 1, on `ardupilot_status_topic` (`/ap/status`, `ardupilot_msgs/msg/Status`) when `drone_type == "ardupilot"`, or `tello_status_topic` (`/tello_state`, `std_msgs/msg/String`) when `drone_type == "tello"`.
4. `_ap_status_callback` (`:583`) records `armed` and `flying` and calls `_reconcile`. `_tello_status_callback` (`:562`) translates `"taking_off"` to `(armed=True, flying=True)` and `"landed"` to `(False, False)` and does the same. Every other Tello state string is ignored, which is a known asymmetry with `video_controller.cpp`, which handles the full enum.
5. `_desired_state` (`:607`) is the policy. Not armed implies `UNCONFIGURED`, armed and flying implies `ACTIVE`, armed and not flying while already `ACTIVE` holds at `ACTIVE`, and armed and not flying otherwise implies `INACTIVE`.
6. `_reconcile` (`:661`) looks up `TRANSITION_STEP`, claims `_transition_in_flight` with a deadline, and `_request_transition` (`:498`) dispatches. `_on_change_state_response` (`:521`) releases the guard and re-enters.
7. `_transition_event_callback` (`:717`) mirrors the state and emits one distinct log line per transition id from `TRANSITION_EVENT_LOGS`.

The hold rule in step 5 is the one judgement call. `flying` on Copter is `!ap.land_complete` and de-asserts one to three seconds after touchdown, roughly half a second before the auto-disarm, so a stateless map would deactivate SLAM just before the disarm that is meant to trigger teardown. Holding `ACTIVE` while armed also stops a land-detector false positive during an aggressive descent from destroying the map mid-flight. Teardown therefore happens on disarm only.

Both entry points, `scripts/basalt_slam_controller` and `scripts/orbslam3_monocular_controller`, spin a `MultiThreadedExecutor`. The status subscription is deliberately left in the node's default, mutually exclusive callback group, so two status messages are never processed at once and the armed/flying pair can never be torn. Every variable shared across threads, `_known_state`, `_transition_in_flight`, `_transition_deadline`, `_last_armed` and `_last_flying`, is guarded by one `threading.RLock`. The lock is reentrant because a future that is already complete can have its done-callback invoked inline on the calling thread by `rclpy`.

`_get_state` and `_change_state` survive at the bottom of the file, marked deprecated, so an out-of-tree caller keeps working. Nothing in this workspace calls them. They must not be reintroduced, because `rclpy.spin_until_future_complete` defaults to the same global executor `rclpy.spin` uses and removes the node from it on the way out, so calling either from inside a callback removes the driver from the executor that is spinning it and no further message is ever delivered. `_change_state` additionally passes no timeout.

### Names to keep consistent

- ORB_SLAM3, compute node `orbslam3_mono_node`, controller name `orbslam3_controller`, `slam_compute_node_name=orbslam3_mono_node`.
- Basalt, compute node `basalt_slam_node`, controller name for example `basalt_controller`, and `slam_compute_node_name=basalt_slam_node` must be set explicitly to override the parent default.

### IMU and VISLAM data pipeline note (for debugging)

Basalt queues IMU internally and `SqrtKeypointVioEstimator::ProcessFrame` pops the relevant IMU on each new frame, so no ROS 2 side timestamp handling is needed. This assumes the camera and IMU streams are time-synchronised with minimal delay. Desync manifests as degraded SLAM tracking rather than as an error.

## Managed-logger lifecycle (`controllers/src/logging/video_controller.cpp`)

`VideoLoggingDriver` drives `managed_logger_node` from the same status topics. Since 2026-07-27 both `ArduPilotStatusCallback` (`:320`) and `TelloStatusCallback` (`:349`) are a decode followed by the same three lines, one guarded call to `UpdateTargetState`, the write of the previous-status flags, and one call to `Reconcile`. The two hundred-line twins are gone.

`UpdateTargetState` (`:208`) is the only place the old branch predicates survive, in their original order and with their lifecycle guards removed.

| Clause | Condition | Latched target |
|---|---|---|
| A | `currentArmed && !mpPreviousArmedStatus` | `ACTIVE` |
| B | `currentFlying && currentArmed` | `ACTIVE` |
| C | `!currentFlying && mpPreviousFlyingStatus` | `UNCONFIGURED` |

There is no `else`, which is what makes the arming edge's effect persist. Clause A is the trigger that matters on both platforms, arming on ArduPilot or `taking_off`/`flying` on Tello, and the reconciler then walks `CONFIGURE` then `ACTIVATE`, so recording starts at the arming edge on both. Clause B is a state-based fallback for a missed arming message, for example a driver started mid-flight or a dropped sample, and it is idempotent because `Reconcile` does nothing when the target already equals the mirror.

`Reconcile` (`:246`) is the mechanism, and `TRANSITION_STEP` in the anonymous namespace (`:33-58`) is its table. That table carries two rows the SLAM driver does not need, `(UNKNOWN, INACTIVE)` and `(UNKNOWN, ACTIVE)`, both mapping to `CONFIGURE`. They are a transcription of the old branches A and B, which both treated `PRIMARY_STATE_UNKNOWN` as equivalent to `UNCONFIGURED`. There is deliberately no `(UNKNOWN, UNCONFIGURED)` row, because the old branch C did not act from `UNKNOWN` either, so a teardown requested before the initial `get_state` has landed remains a no-op.

`OnVideoLoggerChangeStateResponse` (`:414`) no longer spells out the chains. It releases `mpTransitionInFlight`, updates the mirror from the confirmed transition, and re-enters `Reconcile`, which derives the configure-to-activate and deactivate-to-cleanup sequences. `StartCallback` (`:300`) sets the target to `ACTIVE` and reconciles, which removed the last two blocking `SyncCallChangeState` round-trips in the file along with a 2 ms sleep on an executor thread and a transition id of `0` that was passed when no configure was needed. `StopCallback` is still an empty body.

### Three things about this code that are load-bearing

1. The status flags are assigned inside the same critical section that read them, and before any dispatch. `UpdateTargetState` reads `mpPreviousArmedStatus` and `mpPreviousFlyingStatus` and the caller writes them a few lines later under the same lock, which is what makes the arming edge fire exactly once under the `MultiThreadedExecutor` this node uses. Before the reorganisation this ordering also protected the response handler's read of `mpPreviousArmedStatus`, which no longer exists because the latched target subsumes it. If chaining ever stops working, check this ordering first.
2. The initial `get_state` runs from a zero-delay one-shot timer, not the constructor. `main()` only adds the node to the executor after construction returns, so a service response cannot be delivered during the constructor and `wait_for` always burned the full `service_call_timeout_ms` and reported a bogus failure. `QueryInitialState` (`:111`) cancels its own timer and hands the future to `EnqueueServiceResponseHandlerTask`, preserving the `initial_constructor_get_state` context string. `SetupROSInterfaces()` stays in the constructor so subscribers exist before the first spin.
3. `ap_type` and the Tello state strings are matched exactly, and the unmatched case warns. `ap_type` must be lowercase `ardupilot` or `tello`, anything else logs a warning and creates no status subscriber, which disables arm-triggered logging entirely. Previously an unconditional "subscriber created" log fired regardless, which is what hid `ArduPilot` versus `ardupilot` for so long.

### Known gap, teardown is still keyed on the flying edge

Clause C fires on `!currentFlying && mpPreviousFlyingStatus`. Because recording starts on the arming edge, an arm then disarm without ever taking off leaves the logger `ACTIVE` indefinitely, since no flying edge ever falls. The normal land then disarm sequence is unaffected, because landing lowers `flying` first. Closing the gap is now one clause in `UpdateTargetState`, `else if (!currentArmed && mpPreviousArmedStatus)` setting the target to `UNCONFIGURED`, rather than the branch surgery it used to be. It remains deliberately not done, because it is a behavioural change.

Note that the SLAM driver does not have this gap. Its `(INACTIVE, UNCONFIGURED)` step row closes the arm-then-disarm case as a side effect of the table being complete.

### The latent mutex races in these callbacks are closed

Before 2026-07-27 `mpPreviousArmedStatus` and `mpPreviousFlyingStatus` were written under the mutex but read unguarded in the branch conditions, and `mpVideoLoggerKnownState` was written under the mutex but read unguarded in all three branches and in `StartCallback`. Every access to all five guarded members is now inside a `mpcStateMutex` critical section. No path takes any other lock while holding it, so no deadlock is possible, and `Reconcile` takes the lock itself, so no caller may hold it when calling.

## No blocking calls in a node constructor (WORKSPACE RULE, 2026-07-27)

No blocking call may be placed in a node constructor, especially a controller node's. The node must be available to its executor immediately upon construction. Any call that can block belongs in a timer that cancels itself once it has succeeded.

Two independent reasons, both already paid for in this workspace. A service response cannot be delivered during construction, because `main()` adds the node to the executor only after the constructor returns, so a synchronous `wait_for` there always burns its full timeout and reports a bogus failure. And a constructor that blocks makes the whole node invisible for the duration, so `ros2 node list`, parameter queries and every other client on the graph see nothing, which turns a missing dependency into a silent hang rather than a diagnosable wait.

The established pattern is a timer created in the constructor on the reentrant callback group whose callback cancels its own timer and then dispatches asynchronously. Two implementations exist, `VideoLoggingDriver::QueryInitialState` (`video_controller.cpp:53-56, 111-130`), a zero-delay one-shot, and `SLAMDriverNode._await_lifecycle_services` (`driver_node.py:431-456`), a one-second repeating poll that also replaces a `wait_for_service` handshake. Poll with `service_is_ready()`, a non-blocking graph query, rather than `wait_for_service()` inside such a timer.

One violation still stands as of 2026-07-27, `LifecycleControllerBase::WaitForAllRegisteredServices` (`controllers/src/common/controller.cpp:100-140`), called from `VideoLoggingDriver`'s constructor at `video_controller.cpp:46`. Against a missing `managed_logger_node` that constructor never returns. Fixing it needs an additive `bool AllRegisteredServicesReady() const` on the shared base class, which `src/tello/controller.cpp` also derives from, so it was deliberately left out of the 2026-07-27 change and is recorded as a follow-up in [`/ws/plans/slam-lifecycle-ap-status-reconciler.md`](../../plans/slam-lifecycle-ap-status-reconciler.md) §8.

## `transition_event` is the cheap way to mirror a managed node's state

Every `rclcpp_lifecycle::LifecycleNode` publishes `lifecycle_msgs/msg/TransitionEvent` on `~/transition_event` on every state change, including the intermediate transition states. Subscribing to it replaces a per-status-message `get_state` round-trip with a push-based mirror and supplies per-transition diagnostics at the same time. Over the 47 s arm-to-disarm cycle captured in the plan that is roughly 94 service round-trips replaced by about six pushed events.

`SLAMDriverNode` uses it (`driver_node.py:717`). `sensors/src/video/logger.cpp:291-294` is the older in-tree precedent, on `/video_reader/transition_event` with a bare depth of 10. `rcl_lifecycle` creates the publisher with the rcl publisher defaults, `RELIABLE`/`VOLATILE`/`KEEP_LAST(10)`, so a bare depth matches exactly.

Two traps. `msg.transition.label` is not unique, because `rcl_lifecycle` reuses `"transition_success"`, `"transition_failure"` and `"transition_error"` for every outcome transition, so any per-transition dispatch must key on `msg.transition.id`. And a rejected request publishes no event at all, because it never reached a callback, so the `change_state` response is the only record of it and must be logged on its own, which is what `_on_change_state_response` does.

`VideoLoggingDriver` does not use it. Its mirror is still fed by the `change_state` and `get_state` responses. Porting it is the natural follow-up and is now small, because the reorganisation left the mirror written in exactly two places.

See [`rosbag2-recording-architecture.md`](rosbag2-recording-architecture.md) for the recording side, [`slam-package.md`](slam-package.md) for the managed publishers inside the compute nodes, and [`/ws/plans/slam-lifecycle-ap-status-reconciler.md`](../../plans/slam-lifecycle-ap-status-reconciler.md) for the change-record.
