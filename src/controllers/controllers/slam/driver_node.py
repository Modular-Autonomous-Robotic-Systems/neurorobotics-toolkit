#!/usr/bin/env python3
"""Base SLAM driver node: platform/autopilot lifecycle management shared by all SLAM algorithms.

The driver maps the vehicle's flight phase onto the SLAM compute node's managed
lifecycle. It is a target-state reconciler: every status message asks
"where should SLAM be, and what single transition moves it one step closer",
which makes the driver idempotent and self-healing.
"""

import threading

import rclpy
from ardupilot_msgs.msg import Status
from lifecycle_msgs.msg import State, Transition, TransitionEvent
from lifecycle_msgs.srv import ChangeState, GetState
from rcl_interfaces.msg import ParameterDescriptor, ParameterType
from rclpy.node import Node
from std_msgs.msg import String

STATE_NAMES = {
    State.PRIMARY_STATE_UNKNOWN: "UNKNOWN",
    State.PRIMARY_STATE_UNCONFIGURED: "UNCONFIGURED",
    State.PRIMARY_STATE_INACTIVE: "INACTIVE",
    State.PRIMARY_STATE_ACTIVE: "ACTIVE",
    State.PRIMARY_STATE_FINALIZED: "FINALIZED",
    State.TRANSITION_STATE_CONFIGURING: "CONFIGURING",
    State.TRANSITION_STATE_CLEANINGUP: "CLEANINGUP",
    State.TRANSITION_STATE_SHUTTINGDOWN: "SHUTTINGDOWN",
    State.TRANSITION_STATE_ACTIVATING: "ACTIVATING",
    State.TRANSITION_STATE_DEACTIVATING: "DEACTIVATING",
    State.TRANSITION_STATE_ERRORPROCESSING: "ERRORPROCESSING",
}

PRIMARY_STATES = frozenset(
    (
        State.PRIMARY_STATE_UNCONFIGURED,
        State.PRIMARY_STATE_INACTIVE,
        State.PRIMARY_STATE_ACTIVE,
        State.PRIMARY_STATE_FINALIZED,
    )
)

TRANSITION_NAMES = {
    Transition.TRANSITION_CONFIGURE: "CONFIGURE",
    Transition.TRANSITION_CLEANUP: "CLEANUP",
    Transition.TRANSITION_ACTIVATE: "ACTIVATE",
    Transition.TRANSITION_DEACTIVATE: "DEACTIVATE",
    Transition.TRANSITION_UNCONFIGURED_SHUTDOWN: "SHUTDOWN(unconfigured)",
    Transition.TRANSITION_INACTIVE_SHUTDOWN: "SHUTDOWN(inactive)",
    Transition.TRANSITION_ACTIVE_SHUTDOWN: "SHUTDOWN(active)",
}

TRANSITION_STEP = {
    (
        State.PRIMARY_STATE_UNCONFIGURED,
        State.PRIMARY_STATE_INACTIVE,
    ): Transition.TRANSITION_CONFIGURE,
    (
        State.PRIMARY_STATE_UNCONFIGURED,
        State.PRIMARY_STATE_ACTIVE,
    ): Transition.TRANSITION_CONFIGURE,
    (
        State.PRIMARY_STATE_INACTIVE,
        State.PRIMARY_STATE_ACTIVE,
    ): Transition.TRANSITION_ACTIVATE,
    (
        State.PRIMARY_STATE_INACTIVE,
        State.PRIMARY_STATE_UNCONFIGURED,
    ): Transition.TRANSITION_CLEANUP,
    (
        State.PRIMARY_STATE_ACTIVE,
        State.PRIMARY_STATE_INACTIVE,
    ): Transition.TRANSITION_DEACTIVATE,
    (
        State.PRIMARY_STATE_ACTIVE,
        State.PRIMARY_STATE_UNCONFIGURED,
    ): Transition.TRANSITION_DEACTIVATE,
}

TRANSITION_RESULT_STATE = {
    Transition.TRANSITION_CONFIGURE: State.PRIMARY_STATE_INACTIVE,
    Transition.TRANSITION_ACTIVATE: State.PRIMARY_STATE_ACTIVE,
    Transition.TRANSITION_DEACTIVATE: State.PRIMARY_STATE_INACTIVE,
    Transition.TRANSITION_CLEANUP: State.PRIMARY_STATE_UNCONFIGURED,
}

TRANSITION_TIMEOUT_SEC = 10.0

SERVICE_POLL_PERIOD_SEC = 1.0

# ArduCopter's flight modes, transcribed from ArduCopter/mode.h,
COPTER_MODE_NAMES = {
    0: "STABILIZE",
    1: "ACRO",
    2: "ALT_HOLD",
    3: "AUTO",
    4: "GUIDED",
    5: "LOITER",
    6: "RTL",
    7: "CIRCLE",
    9: "LAND",
    11: "DRIFT",
    13: "SPORT",
    14: "FLIP",
    15: "AUTOTUNE",
    16: "POSHOLD",
    17: "BRAKE",
    18: "THROW",
    19: "AVOID_ADSB",
    20: "GUIDED_NOGPS",
    21: "SMART_RTL",
    22: "FLOWHOLD",
    23: "FOLLOW",
    24: "ZIGZAG",
    25: "SYSTEMID",
    26: "AUTOROTATE",
    27: "AUTO_RTL",
    28: "TURTLE",
}

TRANSITION_EVENT_LOGS = {
    Transition.TRANSITION_CREATE: (
        "debug",
        "'{node}' created its lifecycle state machine and is UNCONFIGURED.",
    ),
    Transition.TRANSITION_CONFIGURE: (
        "debug",
        "'{node}' is CONFIGURING, on_configure() has been entered.",
    ),
    Transition.TRANSITION_CLEANUP: (
        "debug",
        "'{node}' is CLEANINGUP, on_cleanup() has been entered.",
    ),
    Transition.TRANSITION_ACTIVATE: (
        "debug",
        "'{node}' is ACTIVATING, on_activate() has been entered.",
    ),
    Transition.TRANSITION_DEACTIVATE: (
        "debug",
        "'{node}' is DEACTIVATING, on_deactivate() has been entered.",
    ),
    Transition.TRANSITION_UNCONFIGURED_SHUTDOWN: (
        "warn",
        "'{node}' is SHUTTINGDOWN from UNCONFIGURED. This driver did not request it, so "
        "something else is tearing the SLAM node down.",
    ),
    Transition.TRANSITION_INACTIVE_SHUTDOWN: (
        "warn",
        "'{node}' is SHUTTINGDOWN from INACTIVE. This driver did not request it, so "
        "something else is tearing the SLAM node down.",
    ),
    Transition.TRANSITION_ACTIVE_SHUTDOWN: (
        "warn",
        "'{node}' is SHUTTINGDOWN from ACTIVE, so SLAM is being stopped mid-flight. This "
        "driver did not request it.",
    ),
    Transition.TRANSITION_DESTROY: (
        "error",
        "'{node}' has destroyed its lifecycle state machine. The compute node is gone and "
        "this driver can no longer control SLAM.",
    ),
    Transition.TRANSITION_ON_CONFIGURE_SUCCESS: (
        "info",
        "CONFIGURE succeeded, '{node}' is now INACTIVE. Parameters have been read and "
        "validated, which for Basalt means the camera and IMU topic names and the "
        "calibration and configuration file paths. No SLAM object exists yet and no image "
        "or IMU callbacks are subscribed. Ready to activate on takeoff.",
    ),
    Transition.TRANSITION_ON_CONFIGURE_FAILURE: (
        "error",
        "CONFIGURE FAILED, on_configure() returned FAILURE and '{node}' fell back to "
        "UNCONFIGURED. SLAM will not run this flight. The most likely cause is "
        "slam_type=VISLAM with an empty imu_topic_name (BasaltSLAMNode::on_configure, "
        "slam/src/basalt/node.cpp:70-75). Check the compute node's log and the launch "
        "arguments.",
    ),
    Transition.TRANSITION_ON_CONFIGURE_ERROR: (
        "error",
        "CONFIGURE RAISED, on_configure() threw or returned ERROR and '{node}' is running "
        "on_error(). This is a code fault rather than a configuration problem, so read the "
        "exception in the compute node's log.",
    ),
    Transition.TRANSITION_ON_ACTIVATE_SUCCESS: (
        "info",
        "ACTIVATE succeeded, '{node}' is now ACTIVE. The SLAM back end has been "
        "constructed, the camera and IMU subscriptions and the TF broadcaster are live, "
        "and pose estimates should start appearing on /tf.",
    ),
    Transition.TRANSITION_ON_ACTIVATE_FAILURE: (
        "error",
        "ACTIVATE FAILED, on_activate() returned FAILURE and '{node}' returned to INACTIVE. "
        "The vehicle is flying without SLAM. Check the calibration and configuration file "
        "paths, because the Basalt back end is constructed at activation rather than at "
        "configuration, so a bad path or an unreadable JSON surfaces here.",
    ),
    Transition.TRANSITION_ON_ACTIVATE_ERROR: (
        "error",
        "ACTIVATE RAISED, on_activate() threw and '{node}' is running on_error(). "
        "Historically this path has meant a native fault inside the Basalt back end, so if "
        "the process died rather than transitioned see "
        "ros_ws/context/simd-eigen-abi-mismatch.md.",
    ),
    Transition.TRANSITION_ON_DEACTIVATE_SUCCESS: (
        "info",
        "DEACTIVATE succeeded, '{node}' is INACTIVE. The camera and IMU subscriptions, the "
        "annotated frame publisher, the TF broadcaster and the SLAM object have been "
        "released, so the map from this flight is gone. Awaiting CLEANUP.",
    ),
    Transition.TRANSITION_ON_DEACTIVATE_FAILURE: (
        "error",
        "DEACTIVATE FAILED, on_deactivate() returned FAILURE and '{node}' is still ACTIVE. "
        "It may still hold the SLAM back end and its subscriptions, so the next arm cycle "
        "will be refused. Restart the compute node.",
    ),
    Transition.TRANSITION_ON_DEACTIVATE_ERROR: (
        "error",
        "DEACTIVATE RAISED, on_deactivate() threw and '{node}' is running on_error(). The "
        "SLAM back end was being destroyed when this happened, so treat any subsequent "
        "crash as related.",
    ),
    Transition.TRANSITION_ON_CLEANUP_SUCCESS: (
        "info",
        "CLEANUP succeeded, '{node}' is UNCONFIGURED and ready for the next arm and takeoff "
        "cycle.",
    ),
    Transition.TRANSITION_ON_CLEANUP_FAILURE: (
        "error",
        "CLEANUP FAILED, on_cleanup() returned FAILURE and '{node}' did not return to "
        "UNCONFIGURED, so the next CONFIGURE on arming will be rejected. SLAM is unavailable "
        "until the compute node is restarted.",
    ),
    Transition.TRANSITION_ON_CLEANUP_ERROR: (
        "error",
        "CLEANUP RAISED, on_cleanup() threw and '{node}' is running on_error(). The node is "
        "not reusable for the next flight.",
    ),
    Transition.TRANSITION_ON_SHUTDOWN_SUCCESS: (
        "warn",
        "'{node}' has shut down cleanly and is FINALIZED. A finalized lifecycle node cannot "
        "be restarted, so this driver can no longer control SLAM.",
    ),
    Transition.TRANSITION_ON_SHUTDOWN_FAILURE: (
        "error",
        "SHUTDOWN FAILED, on_shutdown() returned FAILURE on '{node}'. The node is in an "
        "undefined state and must be restarted.",
    ),
    Transition.TRANSITION_ON_SHUTDOWN_ERROR: (
        "error",
        "SHUTDOWN RAISED, on_shutdown() threw on '{node}'. The node is in an undefined state "
        "and must be restarted.",
    ),
    Transition.TRANSITION_ON_ERROR_SUCCESS: (
        "warn",
        "on_error() handled the fault and '{node}' recovered to UNCONFIGURED. This driver "
        "will re-configure it on the next status message.",
    ),
    Transition.TRANSITION_ON_ERROR_FAILURE: (
        "error",
        "on_error() returned FAILURE on '{node}', which is now FINALIZED and unrecoverable.",
    ),
    Transition.TRANSITION_ON_ERROR_ERROR: (
        "error",
        "on_error() itself threw on '{node}', which is now FINALIZED and unrecoverable.",
    ),
}

RECONCILE_AFTER_TRANSITIONS = frozenset(
    (
        Transition.TRANSITION_ON_CONFIGURE_SUCCESS,
        Transition.TRANSITION_ON_CLEANUP_SUCCESS,
        Transition.TRANSITION_ON_ACTIVATE_SUCCESS,
        Transition.TRANSITION_ON_DEACTIVATE_SUCCESS,
        Transition.TRANSITION_ON_ERROR_SUCCESS,
    )
)


def _mode_name(vehicle_type: int, mode: int) -> str:
    """Human-readable flight mode. Only Copter's enum is transcribed; every other
    vehicle type numbers its modes differently (Status.msg: 'enum depending on
    vehicle type'), so anything else is reported as a bare number."""
    if vehicle_type == Status.APM_ARDUCOPTER:
        return COPTER_MODE_NAMES.get(mode, f"mode {mode}")
    return f"mode {mode}"


class SLAMDriverNode(Node):
    def __init__(self, node_name="mono_py_node"):
        super().__init__(node_name)

        self.declare_parameter("slam_compute_node_name", "orbslam3_mono_node")
        self.declare_parameter("ardupilot_status_topic", "/ap/status")
        self.declare_parameter("tello_status_topic", "/tello_state")
        drone_type_descriptor = ParameterDescriptor(
            type=ParameterType.PARAMETER_STRING,
            description="Type of drone supported by the SLAM driver. Can be one of- 'ardupilot' or 'tello'",
        )
        self.declare_parameter("drone_type", "ardupilot", drone_type_descriptor)

        self.drone_type = str(self.get_parameter("drone_type").value)
        self.slam_compute_node_name = str(
            self.get_parameter("slam_compute_node_name").value
        )
        self.ardupilot_status_topic = str(
            self.get_parameter("ardupilot_status_topic").value
        )
        self.tello_status_topic = str(self.get_parameter("tello_status_topic").value)

        self.get_logger().info(
            f"-------------- Received Autopilot and SLAM parameters --------------------------"
        )
        self.get_logger().info(f"slam_compute_node_name: {self.slam_compute_node_name}")
        self.get_logger().info(f"ardupilot_status_topic: {self.ardupilot_status_topic}")
        self.get_logger().info(f"tello_status_topic: {self.tello_status_topic}")
        self.get_logger().info(f"drone_type: {self.drone_type}")
        self.get_logger().info(
            f"-------------------------------------------------------------"
        )

        self.node_name = node_name

        self.get_logger().info(f"'{self.node_name}' initialized.")
        self.get_logger().info(
            f"Attempting handshake with lifecycle node '{self.slam_compute_node_name}'..."
        )

        # --- Lifecycle Management ---
        self.service_callback_group = rclpy.callback_groups.ReentrantCallbackGroup()
        change_state_service_name = f"/{self.slam_compute_node_name}/change_state"
        get_state_service_name = f"/{self.slam_compute_node_name}/get_state"
        self.change_state_client = self.create_client(
            ChangeState,
            change_state_service_name,
            callback_group=self.service_callback_group,
        )
        self.get_state_client = self.create_client(
            GetState, get_state_service_name, callback_group=self.service_callback_group
        )

        self._state_lock = threading.RLock()
        self._known_state = State.PRIMARY_STATE_UNKNOWN
        self._transition_in_flight = False
        self._transition_deadline = 0.0
        self._last_armed = False
        self._last_flying = False

        transition_event_topic = f"/{self.slam_compute_node_name}/transition_event"
        self.transition_event_subscriber = self.create_subscription(
            TransitionEvent,
            transition_event_topic,
            self._transition_event_callback,
            10,
            callback_group=self.service_callback_group,
        )
        self.get_logger().info(
            f"Observing lifecycle transitions of '{self.slam_compute_node_name}' "
            f"on '{transition_event_topic}'"
        )

        self._service_ready_timer = self.create_timer(
            SERVICE_POLL_PERIOD_SEC,
            self._await_lifecycle_services,
            callback_group=self.service_callback_group,
        )

        # Deliberately left in the node's default, mutually exclusive callback
        # group, so two status messages can never be processed at once and the
        # armed/flying pair can never be torn under the MultiThreadedExecutor.
        status_qos_profile = rclpy.qos.QoSProfile(
            reliability=rclpy.qos.ReliabilityPolicy.BEST_EFFORT,
            history=rclpy.qos.HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        status_topic = ""
        if self.drone_type == "ardupilot":
            status_topic = self.ardupilot_status_topic
            self.status_subscriber = self.create_subscription(
                Status, status_topic, self._ap_status_callback, status_qos_profile
            )
        elif self.drone_type == "tello":
            status_topic = self.tello_status_topic
            self.status_subscriber = self.create_subscription(
                String, status_topic, self._tello_status_callback, status_qos_profile
            )
        self.get_logger().info(
            f"Subscribed to '{status_topic}'. Waiting for messages to control SLAM node..."
        )

    # ****************************************************************************************
    def _await_lifecycle_services(self) -> None:
        """Poll for the compute node's lifecycle services, then seed the state mirror.

        Runs on a repeating timer rather than in the constructor so the node is
        available to its executor immediately. `service_is_ready()` is a
        non-blocking graph query, unlike `wait_for_service()`, so this callback
        never occupies its executor thread for longer than the query itself.
        """
        change_ready = self.change_state_client.service_is_ready()
        get_ready = self.get_state_client.service_is_ready()
        if not (change_ready and get_ready):
            self.get_logger().info(
                f"Waiting for the lifecycle services of '{self.slam_compute_node_name}', "
                f"change_state={'up' if change_ready else 'down'}, "
                f"get_state={'up' if get_ready else 'down'}",
                throttle_duration_sec=10.0,
            )
            return

        self._service_ready_timer.cancel()
        self.get_logger().info(
            f"Lifecycle services of '{self.slam_compute_node_name}' are available, "
            f"querying its initial state"
        )
        self._query_state("initial_handshake", reconcile_after=True)

    # ****************************************************************************************
    def _query_state(self, context: str, reconcile_after: bool) -> None:
        """Ask the compute node for its current state and refresh the mirror.

        `reconcile_after` is False on the re-sync issued after a rejected or
        failed transition. Reconciling there would retry the failed transition
        the instant the answer arrived, which turns a persistent fault such as a
        VISLAM configuration error into a service call storm. The next
        `/ap/status` message retries it instead, which rate-limits recovery to
        the autopilot's 2 Hz keep-alive.
        """
        future = self.get_state_client.call_async(GetState.Request())
        future.add_done_callback(
            lambda fut, ctx=context, rec=reconcile_after: self._on_get_state_response(
                fut, ctx, rec
            )
        )

    # ****************************************************************************************
    def _on_get_state_response(
        self, future, context: str, reconcile_after: bool
    ) -> None:
        """Store the queried state in the mirror, and optionally take the next step."""
        exception = future.exception()
        if exception is not None or future.result() is None:
            self.get_logger().error(
                f"get_state on '{self.slam_compute_node_name}' failed during '{context}', "
                f"{exception}. The lifecycle mirror is stale and will be refreshed on the "
                f"next transition or the next failed request."
            )
            return

        state_id = future.result().current_state.id
        with self._state_lock:
            self._known_state = state_id
        self.get_logger().info(
            f"'{self.slam_compute_node_name}' reports state "
            f"{STATE_NAMES.get(state_id, state_id)} ({context})"
        )
        if reconcile_after:
            self._reconcile()

    # ****************************************************************************************
    def _request_transition(
        self, transition_id: int, believed_state: int, reason: str
    ) -> None:
        """Dispatch one lifecycle transition without blocking the executor.

        The caller must already have claimed `_transition_in_flight` under
        `_state_lock`. This method deliberately performs no service call while
        holding that lock, so a slow round-trip cannot serialise the callback
        groups behind it.
        """
        request = ChangeState.Request()
        request.transition.id = transition_id
        name = TRANSITION_NAMES.get(transition_id, f"transition {transition_id}")
        self.get_logger().info(
            f"Requesting {name} on '{self.slam_compute_node_name}' from "
            f"{STATE_NAMES.get(believed_state, believed_state)}, {reason}"
        )
        future = self.change_state_client.call_async(request)
        future.add_done_callback(
            lambda fut, tid=transition_id: self._on_change_state_response(fut, tid)
        )

    # ****************************************************************************************
    def _on_change_state_response(self, future, transition_id: int) -> None:
        """Handle the change_state reply, release the in-flight guard, take the next step."""
        name = TRANSITION_NAMES.get(transition_id, f"transition {transition_id}")
        with self._state_lock:
            self._transition_in_flight = False
            believed_state = self._known_state

        exception = future.exception()
        if exception is not None or future.result() is None:
            self.get_logger().error(
                f"{name} on '{self.slam_compute_node_name}' raised {exception}. Lifecycle "
                f"control is degraded and SLAM may no longer track the vehicle. "
                f"Re-synchronising the state mirror."
            )
            self._query_state(f"after_{name}_exception", reconcile_after=False)
            return

        if not future.result().success:
            self.get_logger().warn(
                f"{name} was REJECTED by '{self.slam_compute_node_name}'. The driver believed "
                f"the node was {STATE_NAMES.get(believed_state, believed_state)}, but the "
                f"lifecycle state machine does not permit {name} from the node's actual state. "
                f"No transition_event is published for a rejected request, so this line is the "
                f"only record of it. Re-synchronising."
            )
            self._query_state(f"after_{name}_rejected", reconcile_after=False)
            return

        with self._state_lock:
            self._known_state = TRANSITION_RESULT_STATE.get(
                transition_id, self._known_state
            )
        self.get_logger().debug(f"{name} accepted by '{self.slam_compute_node_name}'")
        self._reconcile()

    # ****************************************************************************************
    def _tello_status_callback(self, msg: String):
        """Translate the Tello state string into the same phase the reconciler consumes.

        These spellings are authoritative, they are tello_driver's state_strs_
        map (tello_driver/include/tello_driver_node.hpp). "taking off" with a
        space never matched, so the SLAM node was never configured on takeoff.
        """
        if msg.data == "taking_off":
            armed, flying = True, True
        elif msg.data == "landed":
            armed, flying = False, False
        else:
            return

        with self._state_lock:
            self._last_armed = armed
            self._last_flying = flying
        self.get_logger().info(f"Tello reports state '{msg.data}'")
        self._reconcile()

    # ****************************************************************************************
    def _ap_status_callback(self, msg: Status):
        """Drive the SLAM node's lifecycle from ArduPilot's armed/flying state.

        `/ap/status` publishes on every change and keeps alive at 2 Hz
        (AP_DDS_Client.cpp:761-783), so this is level-triggered rather than
        edge-triggered. The callback records where the vehicle is and asks the
        reconciler to take one step toward where SLAM should therefore be, which
        makes it idempotent and self-healing, so a driver started mid-flight
        converges on the next status message.
        """
        with self._state_lock:
            changed = msg.armed != self._last_armed or msg.flying != self._last_flying
            self._last_armed = msg.armed
            self._last_flying = msg.flying

        if changed:
            self.get_logger().info(
                f"ArduPilot armed={msg.armed} flying={msg.flying} "
                f"mode={_mode_name(msg.vehicle_type, msg.mode)}"
                + (f" failsafe={list(msg.failsafe)}" if msg.failsafe else "")
            )
        self._reconcile()

    # ****************************************************************************************
    def _desired_state(self) -> int:
        """Map the vehicle's phase to the lifecycle state SLAM should be in.

        The caller must hold `_state_lock`.

        ArduCopter forces land_complete (hence flying=False) whenever disarmed
        (ArduCopter/land_detector.cpp:53-55), so (armed=False, flying=True) is
        unreachable and three cases cover the space. Rule R5 of the plan, "not
        armed and not flying implies UNCONFIGURED", is exactly rule R1 under that
        invariant and needs no branch of its own. The cleanup transition R5 calls
        for is supplied by the (INACTIVE, UNCONFIGURED) row of TRANSITION_STEP,
        which is also the only row that can ever emit a CLEANUP.
        """
        if not self._last_armed:
            return State.PRIMARY_STATE_UNCONFIGURED  # R1, and R5
        if self._last_flying:
            return State.PRIMARY_STATE_ACTIVE  # R2
        if self._known_state == State.PRIMARY_STATE_ACTIVE:
            return State.PRIMARY_STATE_ACTIVE
        return State.PRIMARY_STATE_INACTIVE  # R4

    # ****************************************************************************************
    def _phase_description(self, desired: int) -> str:
        """One human sentence explaining why a transition is being requested.

        The caller must hold `_state_lock`, because this reads the recorded
        vehicle phase. It performs no I/O, so holding the lock across it is free.
        """
        if desired == State.PRIMARY_STATE_ACTIVE:
            if self._last_flying:
                return "the vehicle is FLYING, so SLAM must be ACTIVE"
            return (
                "the vehicle is ARMED on the ground and SLAM is already ACTIVE, so SLAM is "
                "held ACTIVE until disarm (rule R3)"
            )
        if desired == State.PRIMARY_STATE_INACTIVE:
            return (
                "the vehicle is ARMED on the ground, so SLAM must be CONFIGURED and ready "
                "for takeoff"
            )
        if desired == State.PRIMARY_STATE_UNCONFIGURED:
            return (
                "the vehicle is DISARMED, so SLAM must be torn down and cleaned up for the "
                "next cycle"
            )
        return f"the target state is {STATE_NAMES.get(desired, desired)}"

    # ****************************************************************************************
    def _reconcile(self) -> None:
        """Take at most one lifecycle step toward the state the vehicle implies.

        The whole decision, that is read the mirror, compute the target, look up
        the step and claim the in-flight guard, happens inside one critical
        section. Splitting it would let two concurrent callbacks read the same
        mirror and dispatch the same transition twice

        The service call itself is issued after the lock is released.
        """
        with self._state_lock:
            if self._transition_in_flight:
                if (
                    self.get_clock().now().nanoseconds * 1e-9
                    < self._transition_deadline
                ):
                    return
                self.get_logger().error(
                    f"A lifecycle transition on '{self.slam_compute_node_name}' has not "
                    f"completed within {TRANSITION_TIMEOUT_SEC:.0f} s. Assuming the response "
                    f"is lost and allowing a retry."
                )
                self._transition_in_flight = False

            known = self._known_state
            if known in (State.PRIMARY_STATE_UNKNOWN, -1):
                return  # initial get_state still pending
            if known == State.PRIMARY_STATE_FINALIZED:
                self.get_logger().error(
                    f"'{self.slam_compute_node_name}' is FINALIZED. A shut-down lifecycle node "
                    f"cannot be restarted, so SLAM is unavailable for the rest of this run. "
                    f"Restart the compute node.",
                    throttle_duration_sec=30.0,
                )
                return

            desired = self._desired_state()
            if desired == known:
                return

            transition = TRANSITION_STEP.get((known, desired))
            if transition is None:
                self.get_logger().warn(
                    f"No single lifecycle step from {STATE_NAMES.get(known, known)} to "
                    f"{STATE_NAMES.get(desired, desired)}, waiting for "
                    f"'{self.slam_compute_node_name}' to settle.",
                    throttle_duration_sec=10.0,
                )
                return

            reason = self._phase_description(desired)
            self._transition_in_flight = True
            self._transition_deadline = (
                self.get_clock().now().nanoseconds * 1e-9 + TRANSITION_TIMEOUT_SEC
            )

        self._request_transition(transition, known, reason)

    # ****************************************************************************************
    def _transition_event_callback(self, msg: TransitionEvent):
        """Log every lifecycle transition of the compute node and mirror its state.

        This subscriber serves two purposes at once. It is the per-transition
        diagnostic log, and it is the push-based replacement for the
        per-status-message get_state round-trip that made the old callback block.
        The mirror is updated before the line is emitted so the two can never
        disagree.
        """
        start = STATE_NAMES.get(msg.start_state.id, str(msg.start_state.id))
        goal = STATE_NAMES.get(msg.goal_state.id, str(msg.goal_state.id))

        if msg.goal_state.id in PRIMARY_STATES:
            with self._state_lock:
                self._known_state = msg.goal_state.id

        severity, text = TRANSITION_EVENT_LOGS.get(
            msg.transition.id,
            (
                "warn",
                f"Unrecognised lifecycle transition id {msg.transition.id} "
                f"(label '{msg.transition.label}') on " + "'{node}'.",
            ),
        )
        getattr(self.get_logger(), severity)(
            f"[{self.slam_compute_node_name}] {start} -> {goal}, "
            + text.format(node=self.slam_compute_node_name)
        )

        if msg.transition.id in RECONCILE_AFTER_TRANSITIONS:
            self._reconcile()

    # ****************************************************************************************
    def _get_state(self, timeout_sec=5.0) -> int:
        """
        Gets the current state of the lifecycle node.

        DEPRECATED
        Use `_query_state` instead, which is asynchronous.

        :param timeout_sec: Timeout for the service call.
        :return: The current state ID (e.g., State.PRIMARY_STATE_UNCONFIGURED) or -1 on failure.
        """
        request = GetState.Request()
        future = self.get_state_client.call_async(request)

        rclpy.spin_until_future_complete(self, future, timeout_sec=timeout_sec)

        if future.result():
            return future.result().current_state.id
        else:
            self.get_logger().error(
                f"Failed to get state of '{self.slam_compute_node_name}' within {timeout_sec}s."
            )
            return -1

    # ****************************************************************************************
    def _change_state(self, transition_id: int) -> bool:
        """
        Calls the change_state service of the lifecycle node to request a state transition.

        DEPRECATED
        Use `_request_transition` instead, which is asynchronous and reports
        its outcome through `_on_change_state_response`.

        :param transition_id: The ID of the transition to request (e.g., Transition.TRANSITION_CONFIGURE).
        :return: True if the transition was successful, False otherwise.
        """
        request = ChangeState.Request()
        request.transition.id = transition_id

        # Call the service asynchronously
        future = self.change_state_client.call_async(request)

        # Wait until the service call is complete
        rclpy.spin_until_future_complete(self, future)

        # Check the result of the service call
        if future.result() is not None:
            if future.result().success:
                return True
            else:
                self.get_logger().warn(f"Transition '{transition_id}' failed.")
                return False
        else:
            self.get_logger().error(
                f"Exception while calling '{self.change_state_client.srv_name}' service: {future.exception()}"
            )
            return False
