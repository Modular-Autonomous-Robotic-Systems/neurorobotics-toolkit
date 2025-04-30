#!/usr/bin/env python3
import rclpy
import time
import threading
import sys
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from lifecycle_msgs.srv import GetState, ChangeState
from lifecycle_msgs.msg import Transition, State, TransitionEvent

# Define transition constants for clarity and easy access
TRANSITION_CONFIGURE = Transition.TRANSITION_CONFIGURE
TRANSITION_CLEANUP = Transition.TRANSITION_CLEANUP
TRANSITION_ACTIVATE = Transition.TRANSITION_ACTIVATE
TRANSITION_DEACTIVATE = Transition.TRANSITION_DEACTIVATE
TRANSITION_UNCONFIGURED_SHUTDOWN = Transition.TRANSITION_UNCONFIGURED_SHUTDOWN
# Add other specific shutdown transitions if needed by your nodes
# TRANSITION_INACTIVE_SHUTDOWN = Transition.TRANSITION_INACTIVE_SHUTDOWN
# TRANSITION_ACTIVE_SHUTDOWN = Transition.TRANSITION_ACTIVE_SHUTDOWN

# Define state constants for clarity and easy access
STATE_UNKNOWN = State.PRIMARY_STATE_UNKNOWN
STATE_UNCONFIGURED = State.PRIMARY_STATE_UNCONFIGURED
STATE_INACTIVE = State.PRIMARY_STATE_INACTIVE
STATE_ACTIVE = State.PRIMARY_STATE_ACTIVE
STATE_FINALIZED = State.PRIMARY_STATE_FINALIZED
TRANSITION_STATE_CONFIGURING = State.TRANSITION_STATE_CONFIGURING
TRANSITION_STATE_SHUTTINGDOWN = State.TRANSITION_STATE_SHUTTINGDOWN
TRANSITION_STATE_ACTIVATING = State.TRANSITION_STATE_ACTIVATING
TRANSITION_STATE_DEACTIVATING = State.TRANSITION_STATE_DEACTIVATING
TRANSITION_STATE_ERRORPROCESSING = State.TRANSITION_STATE_ERRORPROCESSING

# Timeout for service calls (seconds)
DEFAULT_SERVICE_TIMEOUT = 5.0

class VideoTestControllerNode(Node):
    """
    Controls the lifecycle of video_reader and video_logger nodes.

    This node orchestrates the startup (configure, activate) and shutdown
    (deactivate, cleanup, shutdown) of the managed nodes based on lifecycle
    transitions, particularly listening for the video_reader's completion signal.
    It runs the main test sequence in a separate thread.
    """

    def __init__(self):
        """Initializes the VideoTestControllerNode."""
        super().__init__('video_test_controller')
        self.get_logger().info('Initializing Video Test Controller Node...')

        # Use a reentrant callback group allow service calls from callbacks (if needed)
        # and prevent deadlocks if callbacks trigger service calls managed by the same executor.
        self.callback_group = ReentrantCallbackGroup()

        # --- Configuration ---
        self.managed_nodes = ['video_reader', 'video_logger']
        self.lifecycle_clients = {}

        # --- Synchronization Events ---
        # Used to coordinate between the main ROS thread (spinning executor)
        # and the dedicated test sequence thread.
        self._shutdown_requested = threading.Event() # Signals the test thread to start shutdown
        self._test_complete = threading.Event()      # Signals the main thread that the test sequence is done
        self._services_ready = threading.Event()     # Signals that all required services are available

        # --- Create Clients ---
        for node_name in self.managed_nodes:
            self._create_lifecycle_clients(node_name)

        # --- Create Subscriber ---
        # Listens for state changes from the video_reader node
        self.transition_event_sub = self.create_subscription(
            TransitionEvent,
            '/video_reader/transition_event',
            self.transition_event_callback,
            10, # QoS depth
            callback_group=self.callback_group # Use reentrant group
        )

        # --- Start Test Thread ---
        # Run the main test logic in a separate thread to avoid blocking
        # the executor spinning thread.
        # TODO remove the usage of this thread from here and move the call to _run_test_sequence to main method
        self.test_thread = threading.Thread(target=self._run_test_sequence)
        self.test_thread.start()

        self.get_logger().info('Video Test Controller Node initialized.')

    def _create_lifecycle_clients(self, node_name: str):
        """Creates get_state and change_state clients for a given node name."""
        if node_name in self.lifecycle_clients:
            self.get_logger().warn(f'Clients for node {node_name} already exist.')
            return

        self.lifecycle_clients[node_name] = {
            'get_state': self.create_client(
                GetState,
                f'/{node_name}/get_state',
                callback_group=self.callback_group),
            'change_state': self.create_client(
                ChangeState,
                f'/{node_name}/change_state',
                callback_group=self.callback_group)
        }
        self.get_logger().info(f'Created lifecycle clients for node: {node_name}')

    def _wait_for_services(self, timeout_sec: float = 15.0) -> bool:
        """
        Waits for all lifecycle services of managed nodes to become available.

        This method blocks until all services are ready or the timeout expires.
        It should be called from the test thread, not the main ROS thread.
        """
        self.get_logger().info('Waiting for lifecycle services to become available...')
        start_time = self.get_clock().now()
        services_ready = True

        for node_name in self.managed_nodes:
            if not services_ready: break # Stop checking if one failed

            for service_type, client in self.lifecycle_clients[node_name].items():
                self.get_logger().debug(f'Checking service: {client.srv_name}')
                # Loop checking for the specific service
                while not client.wait_for_service(timeout_sec=0.5):
                    elapsed_time = (self.get_clock().now() - start_time).nanoseconds / 1e9
                    if elapsed_time > timeout_sec:
                        self.get_logger().error(
                            f'Service {client.srv_name} for node {node_name} not available '
                            f'after {timeout_sec:.1f} seconds.'
                        )
                        services_ready = False
                        break # Stop waiting for this service

                    if not rclpy.ok(): # Check if ROS is shutting down
                         self.get_logger().warn('ROS shutdown detected while waiting for services.')
                         services_ready = False
                         break

                    self.get_logger().warn(f'Waiting for service {client.srv_name}...')

                if not services_ready: break # Stop checking services for this node

        if services_ready:
            self.get_logger().info('All required lifecycle services are available.')
            self._services_ready.set() # Signal that services are ready
        else:
            self.get_logger().error('One or more lifecycle services failed to become available.')
            self._shutdown_requested.set() # Trigger shutdown if services fail

        return services_ready

    def get_state(self, node_name: str, timeout_sec: float = DEFAULT_SERVICE_TIMEOUT) -> int | None:
        """
        Calls the get_state service for the specified node.

        This method blocks until the service call completes or times out.
        It requires the node's executor to be spinning in a separate thread.

        Args:
            node_name: The name of the managed node.
            timeout_sec: Timeout in seconds for the service call.

        Returns:
            The current state ID (e.g., State.UNCONFIGURED) or None on failure/timeout.
        """
        if node_name not in self.lifecycle_clients:
            self.get_logger().error(f'No clients found for node {node_name}.')
            return None

        client = self.lifecycle_clients[node_name]['get_state']

        # Ensure the service is actually ready before calling
        # wait_for_service can be checked here again for robustness, but
        # _wait_for_services should have ensured this initially.
        if not client.service_is_ready():
             # Attempt a short wait just in case
            if not client.wait_for_service(timeout_sec=1.0):
                self.get_logger().error(f'GetState service for {node_name} is not available.')
                return None

        request = GetState.Request()
        future = client.call_async(request)

        try:
            # spin_until_future_complete requires the executor assigned to the node
            # (and this client's callback group) to be spinning elsewhere.
            self.executor.spin_until_future_complete(future, timeout_sec=timeout_sec)

            if future.done() and future.result() is not None:
                response = future.result()
                state_id = response.current_state.id
                state_label = self._get_state_label(state_id)
                self.get_logger().info(f'Node {node_name} is in state: {state_label} ({state_id})')
                return state_id
            elif future.cancelled():
                 self.get_logger().warn(f'GetState call for {node_name} was cancelled.')
                 return None
            else:
                # Timeout occurred
                self.get_logger().error(
                    f'Failed to get state for node {node_name}: Service call timed out after {timeout_sec} sec.'
                )
                return None
        except Exception as e:
            self.get_logger().error(f'Exception while calling GetState for {node_name}: {e}')
            return None

    def change_state(self, node_name: str, transition_id: int, timeout_sec: float = DEFAULT_SERVICE_TIMEOUT) -> bool:
        """
        Calls the change_state service for the specified node.

        This method blocks until the service call completes or times out.
        It requires the node's executor to be spinning in a separate thread.

        Args:
            node_name: The name of the managed node.
            transition_id: The desired transition ID (e.g., Transition.TRANSITION_CONFIGURE).
            timeout_sec: Timeout in seconds for the service call.

        Returns:
            True if the transition was successful, False otherwise.
        """
        if node_name not in self.lifecycle_clients:
            self.get_logger().error(f'No clients found for node {node_name}.')
            return False

        client = self.lifecycle_clients[node_name]['change_state']
        transition_label = self._get_transition_label(transition_id)

        if not client.service_is_ready():
            if not client.wait_for_service(timeout_sec=1.0):
                self.get_logger().error(f'ChangeState service for {node_name} is not available.')
                return False

        request = ChangeState.Request()
        request.transition.id = transition_id

        self.get_logger().info(f'Requesting transition {transition_label} ({transition_id}) for node {node_name}...')
        future = client.call_async(request)

        try:
            self.executor.spin_until_future_complete(future, timeout_sec=timeout_sec)

            if future.done() and future.result() is not None:
                response = future.result()
                if response.success:
                    self.get_logger().info(f'Node {node_name} successfully triggered transition {transition_label}.')
                    # It takes time for the state to actually change, caller might need to poll get_state
                    return True
                else:
                    self.get_logger().error(f'Node {node_name} failed to trigger transition {transition_label}.')
                    return False
            elif future.cancelled():
                 self.get_logger().warn(f'ChangeState call for {node_name} ({transition_label}) was cancelled.')
                 return False
            else:
                # Timeout occurred
                self.get_logger().error(
                    f'Failed to change state for node {node_name} ({transition_label}): Service call timed out after {timeout_sec} sec.'
                )
                return False
        except Exception as e:
            self.get_logger().error(f'Exception during ChangeState for {node_name} ({transition_label}): {e}')
            return False

    def transition_event_callback(self, msg: TransitionEvent):
        """
        Callback for transition events from the video_reader node.

        Detects if the video_reader has reached a state indicating completion
        (e.g., transitioned from ACTIVE to INACTIVE or reached FINALIZED)
        and triggers the shutdown sequence by setting an event.
        """
        start_label = self._get_state_label(msg.start_state.id)
        if "UNKNOWN" in start_label:
            temp = self._get_transition_label(msg.start_state.id)
            if "UNKNOWN" not in temp:
                start_label = temp
        goal_label = self._get_state_label(msg.goal_state.id)
        if "UNKNOWN" in goal_label:
            temp = self._get_transition_label(msg.goal_state.id)
            if "UNKNOWN" not in temp:
                goal_label = temp
        self.get_logger().info(
            f'Received transition event from video_reader: '
            f'{start_label} -> {goal_label}'
        )

        # --- Shutdown Trigger Logic ---
        # Adjust this condition based on how your video_reader signals completion.
        # Option 1: Transition to INACTIVE signals end of processing.
        # Option 2: Transition to FINALIZED signals end (if node self-finalizes).
        completion_detected = False
        if (msg.start_state.id == TRANSITION_STATE_DEACTIVATING and msg.goal_state.id == STATE_INACTIVE):
            self.get_logger().info("Video reader transitioned Active -> Inactive. Assuming end of stream.")
            completion_detected = True
        elif msg.goal_state.id == STATE_FINALIZED:
            self.get_logger().info("Video reader reached Finalized state. Assuming end of stream.")
            completion_detected = True

        if completion_detected and not self._shutdown_requested.is_set():
            self.get_logger().info("Completion detected. Setting shutdown request event.")
            self._shutdown_requested.set() # Signal the test thread to begin shutdown

    def _perform_transition_for_all(self, transition_id: int, expected_start_state: int | None = None) -> bool:
        """
        Attempts to perform a transition for all managed nodes.

        Checks the expected start state before attempting the transition if provided.

        Args:
            transition_id: The transition to attempt.
            expected_start_state: The state the node must be in before transitioning.

        Returns:
            True if the transition was successfully *triggered* for all relevant nodes,
            False if any service call fails for a node that should transition.
            Note: This doesn't guarantee the nodes *reached* the goal state.
        """
        overall_success = True
        transition_label = self._get_transition_label(transition_id)
        self.get_logger().info(f"--- Attempting transition '{transition_label}' for all nodes ---")

        for node_name in self.managed_nodes:
            node_should_transition = True
            if expected_start_state is not None:
                current_state = self.get_state(node_name) # Blocks until state is retrieved
                if current_state is None:
                    self.get_logger().error(f"Could not get state for {node_name}. Skipping transition.")
                    node_should_transition = False
                    overall_success = False # Getting state failed, count as failure
                elif current_state != expected_start_state:
                    self.get_logger().info(
                        f"Node {node_name} is in state {self._get_state_label(current_state)}, "
                        f"not expected {self._get_state_label(expected_start_state)}. "
                        f"Skipping transition {transition_label}."
                    )
                    node_should_transition = False
                    # Don't mark overall_success=False here, as skipping was intended.
                else:
                     self.get_logger().info(f"Node {node_name} is in expected state {self._get_state_label(expected_start_state)}.")


            if node_should_transition:
                if not self.change_state(node_name, transition_id): # Blocks until service call done
                    self.get_logger().error(f"Failed to trigger transition {transition_label} for node {node_name}.")
                    overall_success = False
                    # Decide if you want to stop on first failure or try all nodes
                    # break # Uncomment to stop on first failure

        if overall_success:
             self.get_logger().info(f"Transition '{transition_label}' triggered successfully where applicable.")
        else:
             self.get_logger().warn(f"One or more nodes failed during transition '{transition_label}'.")

        return overall_success

    def _run_test_sequence(self):
        """
        Executes the main lifecycle state machine for the test.
        This method runs in a dedicated thread (`self.test_thread`).
        """
        self.get_logger().info('Test sequence thread started.')

        # --- 0. Wait for Services ---
        if not self._wait_for_services(timeout_sec=20.0):
             self.get_logger().error("Service check failed. Aborting test sequence.")
             # Ensure shutdown event is set if not already
             self._shutdown_requested.set()
        else:
            # --- 1. Configure Nodes ---
            self.get_logger().info("Step 1: Configuring nodes...")
            if not self._perform_transition_for_all(TRANSITION_CONFIGURE, STATE_UNCONFIGURED):
                self.get_logger().error("Failed to configure all nodes. Initiating shutdown.")
                self._shutdown_requested.set()
            else:
                self.get_logger().info("Nodes configured successfully (transition triggered).")
                time.sleep(1.0) # Allow time for nodes to potentially settle into inactive state

        # --- 2. Activate Nodes ---
        if not self._shutdown_requested.is_set():
            self.get_logger().info("Step 2: Activating nodes...")
            if not self._perform_transition_for_all(TRANSITION_ACTIVATE, STATE_INACTIVE):
                self.get_logger().error("Failed to activate all nodes. Initiating shutdown.")
                self._shutdown_requested.set()
            else:
                self.get_logger().info("Nodes activated successfully (transition triggered). Test is now running.")
                # Nodes are now (presumably) in the Active state.

        # --- 3. Wait for Completion or Shutdown Request ---
        if not self._shutdown_requested.is_set():
            self.get_logger().info("Step 3: Waiting for video reader completion or external shutdown...")
            # Wait until the shutdown event is set (by callback or external signal)
            shutdown_event_processed = self._shutdown_requested.wait(timeout=None) # Wait indefinitely
            if shutdown_event_processed:
                 self.get_logger().info("Shutdown request received.")
            else:
                 # This case should ideally not happen with timeout=None unless wait is interrupted
                 self.get_logger().warn("Wait for shutdown event finished unexpectedly.")


        # --- 4. Shutdown Sequence ---
        # This part executes regardless of whether steps 1/2 succeeded,
        # as long as shutdown was requested (either due to failure, completion, or Ctrl+C).
        self.get_logger().info("Step 4: Starting managed node shutdown sequence...")

        # Deactivate nodes (only attempt if they might be active)
        self.get_logger().info("--- Attempting Deactivation ---")
        self._perform_transition_for_all(TRANSITION_DEACTIVATE, STATE_ACTIVE)
        time.sleep(1.0) # Allow time for state change

        # Cleanup nodes (attempt from inactive state first)
        self.get_logger().info("--- Attempting Cleanup (from Inactive) ---")
        self._perform_transition_for_all(TRANSITION_CLEANUP, STATE_INACTIVE)
        time.sleep(1.0) # Allow time for state change

        # Optionally, attempt cleanup again if nodes might have failed activation
        # and are still unconfigured but need cleanup. Check your node's behavior.
        # self.get_logger().info("--- Attempting Cleanup (from Unconfigured) ---")
        # self._perform_transition_for_all(TRANSITION_CLEANUP, STATE_UNCONFIGURED)
        # time.sleep(1.0)

        # Shutdown nodes (from unconfigured state)
        self.get_logger().info("--- Attempting Shutdown (from Unconfigured) ---")
        self._perform_transition_for_all(TRANSITION_UNCONFIGURED_SHUTDOWN, STATE_UNCONFIGURED)

        self.get_logger().info("Managed node shutdown sequence finished.")
        self.get_logger().info('Test sequence thread finished.')
        self._test_complete.set() # Signal the main thread that we are done

    def join_test_thread(self, timeout_sec: float = 5.0):
        """Waits for the test sequence thread to complete."""
        if self.test_thread.is_alive():
            self.get_logger().info(f"Waiting up to {timeout_sec}s for test thread to join...")
            self.test_thread.join(timeout=timeout_sec)
            if self.test_thread.is_alive():
                 self.get_logger().warn("Test thread did not join within the timeout.")
            else:
                 self.get_logger().info("Test thread joined successfully.")
        else:
             self.get_logger().info("Test thread was already finished.")


    # --- Helper methods for labels ---
    def _get_state_label(self, state_id: int) -> str:
        """Helper to get the string label for a state ID."""
        labels = {v: k for k, v in State.__dict__.items() if k.startswith('PRIMARY_STATE_')}
        return labels.get(state_id, f"UNKNOWN ({state_id})")

    def _get_transition_label(self, transition_id: int) -> str:
        """Helper to get the string label for a transition ID."""
        labels = {v: k for k, v in Transition.__dict__.items() if k.startswith('TRANSITION_')}
        return labels.get(transition_id, f"UNKNOWN ({transition_id})")


def main(args=None):
    rclpy.init(args=args)

    node = None
    executor = None
    try:
        # Use a MultiThreadedExecutor to handle service calls and subscriptions concurrently
        # The test sequence runs in its own thread, but service call futures need the executor.
        executor = MultiThreadedExecutor()
        node = VideoTestControllerNode()
        executor.add_node(node)

        # Keep the main thread alive spinning the executor
        # The test sequence runs in its own thread.
        # We exit when the test thread signals completion via _test_complete event.
        node.get_logger().info("Controller node spinning. Test sequence running in background thread.")
        while rclpy.ok() and not node._test_complete.is_set():
            # Spin the executor to process callbacks (transition events) and service responses
            executor.spin_once(timeout_sec=0.5)

        node.get_logger().info("Test complete signal received or ROS shutting down.")

    except KeyboardInterrupt:
        if node:
            node.get_logger().info('KeyboardInterrupt received, initiating shutdown...')
            # Signal the test thread to start its shutdown sequence if it hasn't already
            node._shutdown_requested.set()
    except Exception as e:
         # Log any unexpected exceptions during initialization or spinning
        if node:
            node.get_logger().fatal(f"Unhandled exception in main: {e}", exc_info=True)
        else:
            print(f"Unhandled exception before node creation: {e}", file=sys.stderr)
    finally:
        # Cleanup
        if node:
            node.get_logger().info('Main thread proceeding to cleanup...')
            # Ensure the test thread finishes its shutdown sequence
            node.join_test_thread()

            node.get_logger().info('Destroying node...')
            # This might cancel pending service calls if the executor stops first
            node.destroy_node()

        if executor:
             # Shut down the executor threads
             executor.shutdown()

        # Shutdown rclpy if it hasn't been already
        if rclpy.ok():
            rclpy.shutdown()

        if node:
             node.get_logger().info('ROS Cleanup complete.') # Logger might not work after shutdown
        print('ROS Cleanup complete.')


if __name__ == '__main__':
    main()
