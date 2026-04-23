"""Launch test for the safety StateMachine."""

import os
import time
import unittest

from ament_index_python.packages import get_package_share_directory
import launch
import launch_testing
import launch_testing.actions
from launch_ros.actions import Node


# colcon build --packages-select exoskeleton_safety_manager
# source install/setup.bash
# colcon test --packages-select exoskeleton_safety_manager --event-handlers console_direct+
# colcon test-result --verbose


def generate_test_description():
    """Launch the state_machine_node with safety params."""
    bringup_pkg = get_package_share_directory('exoskeleton_bringup')
    safety_params = os.path.join(bringup_pkg, 'config', 'safety_params.yaml')

    sm_node = Node(
        package='exoskeleton_safety_manager',
        executable='state_machine_node',
        name='state_machine',
        output='screen',
        parameters=[
            safety_params,
            {
                'bridge_liveness_timeout_sec': 0.5,
                'bridge_liveness_grace_sec': 2.0,
            },
        ],
    )

    return (
        launch.LaunchDescription([
            sm_node,
            launch_testing.actions.ReadyToTest(),
        ]),
        {'state_machine': sm_node},
    )


class TestStateMachineIntegration(unittest.TestCase):
    """Integration tests for the safety StateMachine."""

    @classmethod
    def setUpClass(cls):
        import rclpy
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        import rclpy
        rclpy.shutdown()

    def setUp(self):
        from rclpy.node import Node as RclpyNode
        from std_srvs.srv import SetBool, Trigger
        from std_msgs.msg import String
        from exoskeleton_safety_msgs.srv import SetMode
        from exoskeleton_safety_msgs.msg import SafetyStatus, Float64ArrayStamped

        self.node = RclpyNode('sm_test')

        self.safe_stop_cli = self.node.create_client(
            SetBool, '/safe_stop_request')
        self.compliant_cli = self.node.create_client(
            SetBool, '/compliant_mode_request')
        self.torque_cli = self.node.create_client(
            SetBool, '/torque_limit_request')
        self.reset_cli = self.node.create_client(
            Trigger, '/reset_safety_request')

        self.last_bridge_mode = None
        self.bridge_srv = self.node.create_service(
            SetMode, '/bridge/set_mode', self._bridge_cb)

        self.status_pub = self.node.create_publisher(
            Float64ArrayStamped, '/exo_bridge/status', 10)

        self.last_state = None
        self.node.create_subscription(
            String, '/safety_manager/state',
            lambda m: setattr(self, 'last_state', m.data), 10)

        self.last_status = None
        self.node.create_subscription(
            SafetyStatus, '/safety_manager/status',
            lambda m: setattr(self, 'last_status', m), 10)

        # Wait for SM services to be available
        self.assertTrue(
            self.safe_stop_cli.wait_for_service(timeout_sec=10.0),
            '/safe_stop_request not available')

        # Bring SM to a known state before each test
        self._ensure_fault_monitor()

    def tearDown(self):
        self.node.destroy_node()

    def _bridge_cb(self, req, resp):
        self.last_bridge_mode = req.mode
        resp.success = True
        resp.message = 'mock ok'
        return resp

    # ── Helpers ──────────────────────────────────────────────────

    def _spin(self, sec):
        import rclpy
        end = time.time() + sec
        while time.time() < end:
            rclpy.spin_once(self.node, timeout_sec=0.01)

    def _wait_state(self, expected, timeout=5.0):
        import rclpy
        end = time.time() + timeout
        while time.time() < end:
            rclpy.spin_once(self.node, timeout_sec=0.01)
            if self.last_state == expected:
                return True
        return False

    def _call_bool(self, cli, val):
        import rclpy
        from std_srvs.srv import SetBool
        self.assertTrue(cli.wait_for_service(timeout_sec=5.0))
        req = SetBool.Request()
        req.data = val
        fut = cli.call_async(req)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=5.0)
        return fut.result()

    def _call_reset(self):
        import rclpy
        from std_srvs.srv import Trigger
        self.assertTrue(self.reset_cli.wait_for_service(timeout_sec=5.0))
        req = Trigger.Request()
        fut = self.reset_cli.call_async(req)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=5.0)
        return fut.result()

    def _pub_status(self, mode_id=0.0):
        msg = Float64ArrayStamped()
        msg.header.stamp = self.node.get_clock().now().to_msg()
        msg.data = [
            0.0, 0.0, 0.5, 0.1,
            float('nan'), 0.5, 0.0, mode_id, 0.5,
        ]
        self.status_pub.publish(msg)

    def _feed_bridge(self, n=30, mode_id=0.0, interval=0.01):
        """Feed bridge status to keep liveness alive."""
        for _ in range(n):
            self._pub_status(mode_id)
            self._spin(interval)

    def _expected_mode_id(self):
        """Return the mode_id the SM expects for the current state."""
        mapping = {
            'FAULT_MONITOR': 0.0,
            'COMPLIANT_MODE': 2.0,
            'TORQUE_LIMIT_MODE': 1.0,
            'SAFE_STOP': 3.0,
        }
        return mapping.get(self.last_state, 0.0)

    def _ensure_fault_monitor(self):
        """Reset SM to FAULT_MONITOR from any state.

        This is called in setUp() so each test starts from a clean state.
        Feeds the bridge continuously to prevent liveness timeouts.
        """
        # Feed bridge to keep SM alive while we figure out the state
        self._feed_bridge(n=20, mode_id=0.0)

        if self.last_state is None:
            # First test — wait for SM to initialize
            self._wait_state('FAULT_MONITOR', timeout=10.0)
            self._feed_bridge(n=20, mode_id=0.0)
            return

        if self.last_state == 'FAULT_MONITOR':
            # Already where we want to be
            self._feed_bridge(n=10, mode_id=0.0)
            return

        # If in SAFE_STOP (latched), reset first
        if self.last_state == 'SAFE_STOP':
            self._feed_bridge(n=10, mode_id=3.0)
            self._call_reset()
            self._feed_bridge(n=20, mode_id=0.0)
            self._wait_state('FAULT_MONITOR', timeout=5.0)
            self._feed_bridge(n=10, mode_id=0.0)
            return

        # If in COMPLIANT_MODE, release it
        if self.last_state == 'COMPLIANT_MODE':
            self._feed_bridge(n=10, mode_id=2.0)
            self._call_bool(self.compliant_cli, False)
            self._feed_bridge(n=20, mode_id=0.0)
            self._wait_state('FAULT_MONITOR', timeout=5.0)
            return

        # If in TORQUE_LIMIT_MODE, release it
        if self.last_state == 'TORQUE_LIMIT_MODE':
            self._feed_bridge(n=10, mode_id=1.0)
            self._call_bool(self.torque_cli, False)
            self._feed_bridge(n=20, mode_id=0.0)
            self._wait_state('FAULT_MONITOR', timeout=5.0)
            return

        # Fallback: try reset
        self._feed_bridge(n=10, mode_id=3.0)
        self._call_reset()
        self._feed_bridge(n=20, mode_id=0.0)
        self._wait_state('FAULT_MONITOR', timeout=5.0)

    # ── Tests ────────────────────────────────────────────────────

    def test_01_initial_state(self):
        """SM starts in FAULT_MONITOR, bridge set to nominal."""
        self.assertEqual(self.last_state, 'FAULT_MONITOR')
        self.assertEqual(self.last_bridge_mode, 'nominal')

    def test_02_to_compliant(self):
        """FAULT_MONITOR -> COMPLIANT_MODE."""
        self._feed_bridge(mode_id=0.0)
        r = self._call_bool(self.compliant_cli, True)
        self.assertTrue(r.success)
        self.assertTrue(self._wait_state('COMPLIANT_MODE'))
        self._feed_bridge(mode_id=2.0)
        self.assertEqual(self.last_bridge_mode, 'compliant')

    def test_03_compliant_back(self):
        """COMPLIANT_MODE -> FAULT_MONITOR via release."""
        self._feed_bridge(mode_id=0.0)
        self._call_bool(self.compliant_cli, True)
        self._wait_state('COMPLIANT_MODE')
        self._feed_bridge(mode_id=2.0)

        r = self._call_bool(self.compliant_cli, False)
        self.assertTrue(r.success)
        self._feed_bridge(mode_id=0.0)
        self.assertTrue(self._wait_state('FAULT_MONITOR'))
        self.assertEqual(self.last_bridge_mode, 'nominal')

    def test_04_to_torque_limit(self):
        """FAULT_MONITOR -> TORQUE_LIMIT_MODE."""
        self._feed_bridge(mode_id=0.0)
        r = self._call_bool(self.torque_cli, True)
        self.assertTrue(r.success)
        self.assertTrue(self._wait_state('TORQUE_LIMIT_MODE'))
        self._feed_bridge(mode_id=1.0)
        self.assertEqual(self.last_bridge_mode, 'torque_limit')

    def test_05_to_safe_stop(self):
        """FAULT_MONITOR -> SAFE_STOP."""
        self._feed_bridge(mode_id=0.0)
        r = self._call_bool(self.safe_stop_cli, True)
        self.assertTrue(r.success)
        self.assertTrue(self._wait_state('SAFE_STOP'))
        self._feed_bridge(mode_id=3.0)
        self.assertEqual(self.last_bridge_mode, 'stop')

    def test_06_safe_stop_blocks(self):
        """SAFE_STOP blocks compliant and torque_limit requests."""
        self._feed_bridge(mode_id=0.0)
        self._call_bool(self.safe_stop_cli, True)
        self._wait_state('SAFE_STOP')
        self._feed_bridge(mode_id=3.0)

        r1 = self._call_bool(self.compliant_cli, True)
        self._feed_bridge(mode_id=3.0)
        self.assertFalse(r1.success)

        r2 = self._call_bool(self.torque_cli, True)
        self._feed_bridge(mode_id=3.0)
        self.assertFalse(r2.success)

        self.assertEqual(self.last_state, 'SAFE_STOP')

    def test_07_latch_and_reset(self):
        """Fault latch activates on SAFE_STOP, clears on reset."""
        self._feed_bridge(mode_id=0.0)
        self._call_bool(self.safe_stop_cli, True)
        self._wait_state('SAFE_STOP')
        self._feed_bridge(mode_id=3.0)
        self._spin(0.2)
        self.assertTrue(self.last_status.safe_stop_latched)
        self.assertTrue(self.last_status.fault_present)

        self._feed_bridge(mode_id=3.0)
        r = self._call_reset()
        self.assertTrue(r.success)
        self._feed_bridge(mode_id=0.0)
        self.assertTrue(self._wait_state('FAULT_MONITOR'))
        self._feed_bridge(mode_id=0.0)
        self._spin(0.2)
        self.assertFalse(self.last_status.safe_stop_latched)
        self.assertFalse(self.last_status.fault_present)

    def test_08_no_nominal_transient(self):
        """Bridge mode set by ENTERING plugin, not the exiting one."""
        self._feed_bridge(mode_id=0.0)
        self.last_bridge_mode = None
        self._call_bool(self.compliant_cli, True)
        self._wait_state('COMPLIANT_MODE')
        self._feed_bridge(mode_id=2.0)
        self.assertEqual(self.last_bridge_mode, 'compliant')

    def test_09_bridge_liveness_timeout(self):
        """Bridge death triggers SAFE_STOP via liveness check."""
        # Feed to pass grace period
        for _ in range(30):
            self._pub_status(mode_id=0.0)
            self._spin(0.1)

        # Stop feeding — wait for timeout
        self._spin(1.5)
        self.assertEqual(self.last_state, 'SAFE_STOP')
        self._spin(0.2)
        self.assertFalse(self.last_status.bridge_alive)

    def test_10_mode_mismatch(self):
        """Sustained mode_id mismatch triggers SAFE_STOP."""
        # Feed correct to pass grace period
        for _ in range(30):
            self._pub_status(mode_id=0.0)
            self._spin(0.1)

        # Feed wrong mode_id (3=stop while SM expects 0=nominal)
        for _ in range(20):
            self._pub_status(mode_id=3.0)
            self._spin(0.01)

        self._spin(0.5)
        self.assertEqual(self.last_state, 'SAFE_STOP')

    def test_11_full_cycle(self):
        """Full escalation: FM -> C -> TL -> SS -> reset -> FM."""
        self._feed_bridge(mode_id=0.0)

        self._call_bool(self.compliant_cli, True)
        self._wait_state('COMPLIANT_MODE')
        self._feed_bridge(mode_id=2.0)

        self._call_bool(self.torque_cli, True)
        self._wait_state('TORQUE_LIMIT_MODE')
        self._feed_bridge(mode_id=1.0)

        self._call_bool(self.safe_stop_cli, True)
        self._wait_state('SAFE_STOP')
        self._feed_bridge(mode_id=3.0)

        r = self._call_reset()
        self.assertTrue(r.success)
        self._feed_bridge(mode_id=0.0)
        self.assertTrue(self._wait_state('FAULT_MONITOR'))
        self.assertEqual(self.last_bridge_mode, 'nominal')

    def test_12_bridge_alive_field(self):
        """bridge_alive is true when bridge is feeding status."""
        for _ in range(30):
            self._pub_status(mode_id=0.0)
            self._spin(0.1)
        self._spin(0.2)
        self.assertTrue(self.last_status.bridge_alive)
        self.assertTrue(self.last_status.bridge_mode_coherent)


@launch_testing.post_shutdown_test()
class TestShutdown(unittest.TestCase):
    """Verify clean shutdown."""

    def test_exit_code(self, proc_info):
        launch_testing.asserts.assertExitCodes(proc_info)