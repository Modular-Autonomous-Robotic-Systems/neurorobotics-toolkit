# rosbag2 recording in this workspace: `ManagedLogger`, the `Recorder` API, and how latched messages are captured

**Date:** 2026-07-25, revised the same day once the fixes landed. **Status:** the two defects described under "Capturing the *trigger* message" are **fixed**; everything else is durable API description. **Scope:** `sensors` (owns the bag) and `controllers` (drives its lifecycle). **ROS distro:** Humble; all API references are to the `humble` branch of [ros2/rosbag2](https://github.com/ros2/rosbag2).
**Related:** [`ros-qos-compatibility.md`](ros-qos-compatibility.md) (subscriber QoS matrix), [`/ws/context/ardupilot-dds-qos.md`](../../context/ardupilot-dds-qos.md) (publisher QoS), [`slam-lifecycle-flow.md`](slam-lifecycle-flow.md).

## The architecture

Recording is split across two nodes in two processes:

- **`ManagedLogger`** (`sensors/src/logger/managed_logger.cpp`, node name `managed_logger_node`) is an `rclcpp_lifecycle::LifecycleNode` that owns the bag. It does **not** subscribe to anything itself. On `on_configure` it builds `StorageOptions` (`uri`, `storage_id = "sqlite3"`) and `RecordOptions` (`all = false`, `topics`, `rmw_serialization_format`). On `on_activate` it creates a `rosbag2_cpp::Writer` via `rosbag2_transport::ReaderWriterFactory::make_writer()`, wraps it in a `rosbag2_transport::Recorder` named `managed_logger_node_internal`, spins that recorder on its **own** `MultiThreadedExecutor` in a **dedicated thread**, and calls `Recorder::record()`. `on_deactivate` calls `Recorder::stop()`, cancels the executor, joins the thread and removes the node.
- **`VideoLoggingDriver`** (`controllers/src/logging/video_controller.cpp`, node name `video_logging_controller`) is a plain `rclcpp::Node` that drives `managed_logger_node` through `configure`/`activate`/`deactivate`/`cleanup` in response to `/ap/status` (`armed`, `flying`) or a message on `start_topic`.

So the bag's lifecycle is controlled from a different process than the one that owns it — which is the constraint behind every design question below.

## Initialisation: what happens, in what order, when the logger activates

Every timing conclusion in this document follows from this sequence.

**0. `main()` starts the node and its executor, before any transition.** `managed_logger.cpp:244-254` constructs `ManagedLogger`, adds it to a `MultiThreadedExecutor` and spins. `rclcpp_lifecycle::LifecycleNode` places the node in `PRIMARY_STATE_UNCONFIGURED` and advertises `~/change_state` / `~/get_state` automatically. **`on_configure` and `on_activate` therefore never run "at startup" — they run later, as service callbacks dispatched by this already-spinning executor.** Two consequences: a transition callback can block without starving the node (the executor is multi-threaded), and the `~/change_state` response is sent only *after* the transition callback returns, because `rclcpp_lifecycle` invokes it synchronously inside the service handler. That ordering is what makes configure→activate chaining well-defined rather than a race.

**1. `on_configure` — builds options, opens nothing.** `managed_logger.cpp:54-68` fills `StorageOptions` (`uri`, `storage_id = "sqlite3"`; `snapshot_mode` `false`, `max_cache_size`/`max_bagfile_size`/`max_bagfile_duration` all `0`) and `RecordOptions` (`all = false`, `topics`, `rmw_serialization_format`). Defaults that matter and are left alone: `topic_polling_interval` **100 ms**, `is_discovery_disabled` `false`, `start_paused` `false`, `use_sim_time` `false`, `topic_qos_profile_overrides` **empty**. No bag file, no subscription.

**2. `on_activate` — builds writer and recorder node.** `ReaderWriterFactory::make_writer(RecordOptions)` returns a `rosbag2_cpp::Writer` over a `SequentialWriter` (or a compression-aware writer if `compression_format`/`compression_mode` were set — they are not). The `Recorder` constructor only stores writer + options; it is a plain `rclcpp::Node` named `managed_logger_node_internal`. Constructing it has no graph side effects beyond the node itself.

**3. A *second*, dedicated executor and thread for the recorder.** `managed_logger.cpp:105-109` creates its own `MultiThreadedExecutor`, adds the `Recorder` node and spins it on a `std::thread`. Do not confuse it with the executor from step 0: `ManagedLogger` is spun by `main()`'s executor, the `Recorder` by this one. Hence the separate entry in `ros2 node list`, and bag writes landing on a different thread from `ManagedLogger`'s lifecycle callbacks.

**4. `Recorder::record()` — the real work, in this order:**
   1. `writer_->open(storage_options_, {rmw_get_serialization_format(), record_options_.rmw_serialization_format})` — **now** the bag directory and `metadata.yaml` appear.
   2. `~/snapshot` service created **only if** `storage_options_.snapshot_mode` — it is not, so this stack has none.
   3. `events/write_split` publisher and its event thread start.
   4. `subscribe_topics(get_requested_or_available_topics())` — asks the graph for topics that exist **right now** and filters them against `record_options_.topics`. **A requested topic with no publisher yet is silently skipped, not an error.** (Skipped entirely when `use_sim_time` is set — then it waits for `/clock` first.)
   5. `topics_discovery()` thread launches, re-polling every `topic_polling_interval` (100 ms) and subscribing to whatever appeared since.

**5. `subscribe_topic()` per topic:** `writer_->create_topic(topic)` **first** — so the topic exists in the bag even if no message ever arrives — then `create_generic_subscription()` with a negotiated QoS, and a callback doing `writer_->write(message, topic_name, topic_type, this->get_clock()->now())` unless paused.

**The governing consequence.** The earliest instant any message can be captured is step 4.4 — subscription creation, *inside* the activate transition. **Everything published before that is lost unless the publisher retains it via `TRANSIENT_LOCAL`.** A trigger message is by definition published before the activation it triggers, so capturing it is purely a question of publisher durability, or of writing it in by hand.

## How the `Recorder` chooses subscription QoS — and why it matters

`Recorder::subscribe_topic()` does **not** use a fixed QoS. It calls `subscription_qos_for_topic()`, which first looks for an entry in `topic_qos_profile_overrides_` and, finding none, delegates to `Rosbag2QoS::adapt_request_to_offers()` (`rosbag2_transport/src/rosbag2_transport/qos.cpp`). That function inspects every publisher endpoint on the topic and applies an all-or-nothing rule to each policy:

```cpp
if (reliability_reliable_endpoints_count == num_endpoints) { request_qos.reliable(); }
else { request_qos.best_effort(); }        // "will connect to all publishers"

if (durability_transient_local_endpoints_count == num_endpoints) { request_qos.transient_local(); }
else { request_qos.durability_volatile(); } // "will connect to all publishers"
```

**Consequence for `/ap/status`.** AP_DDS publishes it as `TRANSIENT_LOCAL` + `RELIABLE` + `KEEP_LAST(1)` — the only latched topic in the `/ap/*` set — and is the only publisher, so `1 == 1` for both policies and the recorder requests transient-local. DDS therefore delivers the **retained sample at subscription time**, which is genuinely useful: the recorder starts with a status message rather than waiting for the next one.

**But it does not capture the arming trigger, and it does nothing for Tello.** See the next section — this was over-claimed in an earlier revision of this file and the correction matters.

Two things can silently break this:

1. **A `topic_qos_profile_overrides` entry bypasses the adaptation entirely.** `managed_logger.cpp:71-84` still carries a **commented-out** block applying `VOLATILE` + `BEST_EFFORT` to *every* recorded topic, self-described as making the recorder "compatible with `TRANSIENT_LOCAL` publishers". It does the opposite: uncommenting it would discard the latched sample on both trigger topics and defeat the whole trigger-capture fix. It was left commented rather than deleted by explicit decision — **do not uncomment it**. Never apply a blanket override; if a specific topic needs one, scope it to that topic and record why.
2. **A second publisher on the topic.** If anything else ever publishes `/ap/status` — a test fixture, a `ros2 bag play` replay running alongside SITL — the transient-local count no longer equals the endpoint count and the recorder falls back to `VOLATILE`, losing the latched sample.

Verify at runtime with `ros2 topic info -v /ap/status`: the `managed_logger_node_internal` subscriber must report `Durability: TRANSIENT_LOCAL`.

## Capturing the *trigger* message: ArduPilot and Tello are not symmetric

The two supported platforms behave completely differently here, and the difference is not visible from the ROS side alone.

| | ArduPilot | Tello |
|---|---|---|
| Trigger topic | `/ap/status` (`ardupilot_msgs/msg/Status`) | `/<ns>/tello_state` (`std_msgs/msg/String`) |
| Publisher | AP_DDS, `AP_DDS_Topic_Table.h` | `tello_driver/src/tello_driver_node.cpp` |
| Durability | **TRANSIENT_LOCAL**, depth 1 | **TRANSIENT_LOCAL**, depth 1 since 2026-07-25 (`rclcpp::QoS(1).transient_local()`); was VOLATILE |
| Republished? | keep-alive every **500 ms** (`AP_DDS_Client.cpp:762-782`; `AP_DDS_config.h:110` sets `DELAY_STATUS_TOPIC_MS = 100`, republish at ×5) | **Never.** `update_tello_state()` (`tello_driver_node.cpp:210-241`) publishes once per state edge. No timer. |
| Late subscriber gets the last sample? | Yes | Yes, since 2026-07-25 (**No** before) |

**ArduPilot, before the fix: the latched sample existed but was the wrong one.** Depth is 1, so only the most recent status is retained, and the lifecycle used *two* separate events:

- `armed` → true fires the first branch of `ArduPilotStatusCallback` → **CONFIGURE**. The chain-to-activate test in `OnVideoLoggerChangeStateResponse` is `if (mpPreviousFlyingStatus && mpPreviousArmedStatus)`, and `mpPreviousFlyingStatus` is still false — so **activation does not happen on arming**.
- `flying` → true (takeoff) fires the second branch → **ACTIVATE** → the recorder subscribes.

By subscription time the retained depth-1 sample was the **flying** status; the **arming** status had been superseded and was gone. So ArduPilot recorded the *activation* trigger, never the *arming* trigger.

**Tello, before the fix: nothing was captured.** The publisher was volatile and never republishes, so the state message that triggered recording was unrecoverable the instant it was sent. The topic appeared in the bag (because `subscribe_topic()` calls `create_topic()` first) with zero messages until the next state edge.

**Tello does chain configure → activate, though.** `TelloStatusCallback` sets `currentArmed` and `currentFlying` from a *single* message, so both flags are true when the configure response lands and the activate genuinely chains. This is the asymmetry that makes a mental model derived from Tello wrong for ArduPilot, and vice versa.

**Two independent defects, one per platform, each fixed in a few lines (2026-07-25).** ArduPilot **latched but activated too late**; Tello **activated promptly but did not latch**. The resolution was symmetric:

1. **Chain `configure` → `activate` on the arming edge**, by relaxing the chain test in `OnVideoLoggerChangeStateResponse` from `mpPreviousFlyingStatus && mpPreviousArmedStatus` to `mpPreviousArmedStatus` alone. The recorder now subscribes tens of milliseconds after the arming message, while it is still the retained sample. **This required moving the `mpPreviousArmedStatus`/`mpPreviousFlyingStatus` assignments *above* the async dispatch** in both status callbacks — they used to be assigned after it, and the response handler runs on a worker thread that could observe the stale value and silently skip the chain. That ordering is load-bearing; if the chain ever stops firing, check it first.
2. **Latch `tello_state`**: `create_publisher<std_msgs::msg::String>("tello_state", rclcpp::QoS(1).transient_local())` in `tello_driver_node.cpp`. `TRANSIENT_LOCAL` is a stronger *offer*, so every existing volatile subscriber keeps matching — backward-compatible, and it also makes `ros2 topic echo` on that topic useful for the first time.

**The timing margin.** On ArduPilot the chain is two local service round-trips plus bag creation — order tens of milliseconds, dominated by `writer_->open()`. The next `/ap/status` is either the 500 ms keep-alive (identical content, since `update_topic()` republishes only on change or on the timer) or the flying edge, seconds away. So the retained sample at subscription time still reads `armed=1, flying=0`. On Tello there is no deadline at all once latched. The one way to erode the ArduPilot margin is a slow `on_configure` — `UpdateFilenameIfExists` recursing over a directory with very many existing bags is the realistic candidate.

No new service, parameter or interface package was needed. An earlier revision proposed injecting the cached trigger through `mpWriter` via a `~/log_message` service; it was withdrawn once it was clear the lifecycle simply activated too late. The `Writer` API notes below are retained because they remain the right tool if a future requirement needs *arbitrary* synthetic messages written into a bag.

## A related string-matching defect on the Tello path

`tello_driver_node.hpp:59-66` defines the authoritative state strings: `idle`, `landed`, **`taking_off`**, `flying`, `landing`, `low_battery`. Both consumers used to compare against `"taking off"` **with a space** — `video_controller.cpp` and `driver_node.py` — so the takeoff edge never matched in either the recording or the SLAM lifecycle path. `landing`, `idle` and `low_battery` had no branch at all and fell through to the "not flying" path, tearing recording down early. **Fixed 2026-07-25** in both files: exact `taking_off`, an explicit `landing` branch that keeps recording through touchdown (teardown happens on `landed`), `landed`/`idle` tearing down, `low_battery` warning without changing state, and an unrecognised-string warning. `tello_gazebo/src/tello_plugin.cpp:70-73` uses the same spellings but does **not** publish `tello_state`, so `tello_driver` is the sole publisher. Same family as the `ap_type` `"ArduPilot"` vs `"ardupilot"` defect; see [`/ws/plans/basalt-slam-defect-remediation.md`](../../plans/basalt-slam-defect-remediation.md) Change 3.6.

## `rosbag2_cpp::Writer` — the API for injecting a message by hand

Useful when the exact message instance and its original receipt timestamp must be preserved.

- **`Writer` is thread-safe.** Every `write()` overload and `create_topic()` opens with `std::lock_guard<std::mutex> writer_lock(writer_mutex_);`. Writing from another thread while the `Recorder` writes from its own executor thread is explicitly supported.
- **`ManagedLogger` already holds the writer** — `mpWriter` (`managed_logger.h:69`), a non-const `std::shared_ptr<rosbag2_cpp::Writer>`. This is the only usable route: `Recorder::get_writer_handle()` returns a **`const rosbag2_cpp::Writer &`** and cannot be written through.
- **The convenient overload auto-creates the topic.** `write(std::shared_ptr<const rclcpp::SerializedMessage>, topic_name, type_name, const rclcpp::Time &)` builds a `SerializedBagMessage`, sets `time_stamp = time.nanoseconds()` from the caller's value, then delegates to the `SerializedBagMessage` overload, which **calls `create_topic()` itself**. So the caller fully controls the recorded timestamp and the topic need not already exist in the bag.
- There is also a template overload `write(const MessageT &, topic_name, const rclcpp::Time &)` that serialises automatically — simpler when the typed message is in hand.
- Out-of-order timestamps are accepted by the `sqlite3` storage plugin this stack uses; `ros2 bag info` derives duration from min/max, so an injected earlier timestamp just widens the reported window.

**Cross-process caveat.** `VideoLoggingDriver` runs in a different process and cannot touch `mpWriter`, so injecting from the controller would need an IPC hop (a service on `ManagedLogger` forwarding to `mpWriter->write(...)`). This is why injection lost to the lifecycle fix for the trigger-message problem — but it is still the route for any genuinely synthetic message that no publisher produces.

## `Recorder` control surface, and what is *not* available by default

| Feature | Availability here | Notes |
|---|---|---|
| `record()` / `stop()` | Used | `stop()` dumps buffers and closes the writer. |
| `pause()` / `resume()` / `toggle_paused()` / `is_paused()` | Available, unused | The subscription callback checks the paused flag and drops messages while paused. |
| `~/snapshot` service (`rosbag2_interfaces::srv::Snapshot`) | **Not created** | `Recorder` only advertises it when `storage_options_.snapshot_mode` is true. `managed_logger.cpp` sets only `uri` and `storage_id`, so snapshot mode is off. |
| `events/write_split` publisher | Created | `rosbag2_interfaces::msg::WriteSplitEvent`, emitted on bag split. Unused here since `max_bagfile_size`/`max_bagfile_duration` are left at 0. |
| `topics_using_fallback_qos()` | Available, unused | Would report exactly the topics where `adapt_request_to_offers()` had to degrade — a good diagnostic for the failure mode above. |

**Do not use pause/resume to defer recording start.** [ros2/rosbag2#1302](https://github.com/ros2/rosbag2/issues/1302) documents that starting paused loses latched topics entirely (reported for `/tf_static` recording zero messages): the retained sample is delivered on subscription while recording is still paused, and is dropped. That is precisely the message a "capture the trigger" design is trying to keep.

**Snapshot mode is the right tool for a different job.** With `storage_options.snapshot_mode = true` and a `max_cache_size`, the recorder buffers into a circular cache and writes to disk *only* when `~/snapshot` is called — giving pre-trigger history. Excellent for a future "keep the 30 s before arming" feature; wrong for continuous post-arm recording, which would need repeated triggers.

## Gotchas worth remembering

- **`ros2 node info` cannot tell you whether recording will work.** It shows a subscription exists, not what QoS it negotiated or whether it matched. `ros2 topic info -v` is the diagnostic.
- **The recorder node is a separate node from the lifecycle node.** It is named `<managed_logger_node>_internal` and spun on its own executor in its own thread — so it appears separately in `ros2 node list`, and killing or inspecting `managed_logger_node` alone can mislead.
- **`ManagedLogger::UpdateFilenameIfExists`** (`managed_logger.cpp:185-225`) auto-increments the bag path when it already exists, so a re-run does not overwrite — but it recurses and mutates by string surgery on the last `_`, which makes the resulting name hard to predict. Check the "Provided filename exists, logging at:" log line for the actual path before hunting for a missing bag.
- **`sensors/package.xml` used to misspell two dependencies** — `lifecyle_msgs` (`:16`, `:27`) and `rosbag2_trasport` (`:18`). `rosdep` could resolve neither, so they were never installed by a rosdep pass; the package built only because the correct packages were already in the image. **Fixed 2026-07-25.** The general lesson stands: a typo'd rosdep key is invisible to `rosdep install --simulate`, so it under-reports rather than erroring.

## Sources

- [ros2/rosbag2 (humble)](https://github.com/ros2/rosbag2) — `rosbag2_cpp/{include,src}/rosbag2_cpp/writer.{hpp,cpp}`, `rosbag2_transport/{include,src}/rosbag2_transport/recorder.{hpp,cpp}`, `rosbag2_transport/src/rosbag2_transport/qos.cpp`, `rosbag2_storage/include/rosbag2_storage/storage_options.hpp`.
- [ros2/rosbag2#1302 — record with `--start-paused` ignores `/tf_static`](https://github.com/ros2/rosbag2/issues/1302).
- [ros2/rosbag2#1159 — support `--repeat-latched` (transient local)](https://github.com/ros2/rosbag2/issues/1159) and [#129 — support for latching](https://github.com/ros2/rosbag2/issues/129).
- [ros2/ros2#464 — improve latching/transient_local support in ROS 2](https://github.com/ros2/ros2/issues/464).
