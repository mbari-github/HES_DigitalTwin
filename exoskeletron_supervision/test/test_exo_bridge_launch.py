"""Launch test for the ExoBridge safety gateway."""

import os
import time
import unittest

from ament_index_python.packages import get_package_share_directory
import launch
import launch_testing
import launch_testing.actions
from launch_ros.actions import Node

# How to use this tester:
# cd ros2_ws
# source install/setup.bash
# launch_test src/HES_DigitalTwin/exoskeletron_supervision/test/test_exo_bridge_launch.py



def generate_test_description():
    """Launch the exo_bridge node with params."""
    bringup_pkg = get_package_share_directory('exoskeletron_bringup')
    bridge_params = os.path.join(
        bringup_pkg, 'config', 'exo_bridge_params.yaml')

    bridge_node = Node(
        package='exoskeletron_supervision',
        executable='exo_bridge',
        name='exo_bridge',
        output='screen',
        parameters=[bridge_params],
    )

    return (
        launch.LaunchDescription([
            bridge_node,
            launch_testing.actions.ReadyToTest(),
        ]),
        {'exo_bridge': bridge_node},
    )


class TestExoBridgeIntegration(unittest.TestCase):
    """Integration tests for the ExoBridge."""

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
        from std_msgs.msg import Float64, Float64MultiArray, String, Bool
        from std_srvs.srv import SetBool
        from sensor_msgs.msg import JointState
        from exoskeletron_safety_msgs.srv import SetMode

        self.node = RclpyNode('bridge_test')

        self.pub_js = self.node.create_publisher(
            JointState, '/joint_states', 10)
        self.pub_traj = self.node.create_publisher(
            Float64MultiArray, '/trajectory_ref_raw', 10)
        self.pub_tau = self.node.create_publisher(
            Float64, '/torque_raw', 10)
        self.pub_ext = self.node.create_publisher(
            Float64, '/exo_dynamics/tau_ext_theta', 10)

        self.last_status = None
        self.last_mode = None
        self.last_torque_out = None
        self.last_freeze = None

        self.node.create_subscription(
            Float64MultiArray, '/exo_bridge/status',
            lambda m: setattr(self, 'last_status', list(m.data)), 10)
        self.node.create_subscription(
            String, '/exo_bridge/mode',
            lambda m: setattr(self, 'last_mode', m.data), 10)
        self.node.create_subscription(
            Float64, '/torque',
            lambda m: setattr(self, 'last_torque_out', m.data), 10)
        self.node.create_subscription(
            Bool, '/admittance/freeze',
            lambda m: setattr(self, 'last_freeze', m.data), 10)

        self.set_mode_cli = self.node.create_client(
            SetMode, '/bridge/set_mode')

        self.safe_stop_received = False
        self.node.create_service(
            SetBool, '/safe_stop_request', self._mock_safe_stop)

        self.assertTrue(
            self.set_mode_cli.wait_for_service(timeout_sec=10.0),
            '/bridge/set_mode not available')

        self._request_mode('nominal')
        self._feed_all(n=20)

    def tearDown(self):
        self.node.destroy_node()

    def _mock_safe_stop(self, req, resp):
        self.safe_stop_received = True
        resp.success = True
        resp.message = 'mock ok'
        return resp

    def _spin(self, sec):
        import rclpy
        end = time.time() + sec
        while time.time() < end:
            rclpy.spin_once(self.node, timeout_sec=0.01)

    def _request_mode(self, mode):
        import rclpy
        from exoskeletron_safety_msgs.srv import SetMode
        req = SetMode.Request()
        req.mode = mode
        fut = self.set_mode_cli.call_async(req)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=5.0)
        return fut.result()

    def _pub_all(self, tau=1.0, theta=0.5, theta_dot=0.1):
        from std_msgs.msg import Float64, Float64MultiArray
        from sensor_msgs.msg import JointState

        js = JointState()
        js.header.stamp = self.node.get_clock().now().to_msg()
        js.name = ['rev_crank']
        js.position = [theta]
        js.velocity = [theta_dot]
        js.effort = [0.0]
        self.pub_js.publish(js)

        traj = Float64MultiArray()
        traj.data = [theta, theta_dot, 0.0]
        self.pub_traj.publish(traj)

        tau_msg = Float64()
        tau_msg.data = tau
        self.pub_tau.publish(tau_msg)

        ext_msg = Float64()
        ext_msg.data = 0.0
        self.pub_ext.publish(ext_msg)

    def _feed_all(self, n=30, tau=1.0):
        for _ in range(n):
            self._pub_all(tau=tau)
            self._spin(0.01)

    # ── 1. Mode transitions ─────────────────────────────────────

    def test_01_initial_mode_nominal(self):
        """Bridge starts in nominal mode."""
        self._feed_all()
        self.assertIsNotNone(self.last_mode)
        self.assertEqual(self.last_mode.strip().lower(), 'nominal')

    def test_02_to_compliant(self):
        """Transition to compliant mode."""
        r = self._request_mode('compliant')
        self.assertTrue(r.success)
        self._feed_all()
        self.assertEqual(self.last_mode.strip().lower(), 'compliant')

    def test_03_to_torque_limit(self):
        """Transition to torque_limit mode."""
        r = self._request_mode('torque_limit')
        self.assertTrue(r.success)
        self._feed_all()
        self.assertEqual(self.last_mode.strip().lower(), 'torque_limit')

    def test_04_to_stop(self):
        """Transition to stop mode."""
        r = self._request_mode('stop')
        self.assertTrue(r.success)
        self._feed_all()
        self.assertEqual(self.last_mode.strip().lower(), 'stop')

    def test_05_invalid_mode(self):
        """Invalid mode is rejected."""
        r = self._request_mode('turbo')
        self.assertFalse(r.success)

    def test_06_full_cycle(self):
        """nominal -> compliant -> torque_limit -> stop -> nominal."""
        for mode in ['compliant', 'torque_limit', 'stop', 'nominal']:
            r = self._request_mode(mode)
            self.assertTrue(r.success, f'Failed: {mode}')
            self._feed_all(n=5)

    # ── 2. Status layout ────────────────────────────────────────

    def test_07_status_9_fields(self):
        """Status message has exactly 9 fields."""
        self._feed_all(n=30, tau=1.5)
        self.assertIsNotNone(self.last_status)
        self.assertEqual(len(self.last_status), 9)

    def test_08_mode_id_nominal(self):
        """mode_id = 0 in nominal."""
        self._feed_all()
        self.assertIsNotNone(self.last_status)
        self.assertEqual(self.last_status[7], 0.0)

    def test_09_mode_id_stop(self):
        """mode_id = 3 and is_stop = 1 in stop mode."""
        self._request_mode('stop')
        self._feed_all()
        self.assertEqual(self.last_status[0], 1.0)
        self.assertEqual(self.last_status[7], 3.0)

    def test_10_tau_raw_pre_clamp(self):
        """data[8] is pre-clamp tau, data[5] is post-clamp."""
        self._request_mode('torque_limit')
        self._feed_all(n=30, tau=5.0)
        self.assertIsNotNone(self.last_status)
        tau_out = self.last_status[5]
        tau_raw = self.last_status[8]
        self.assertAlmostEqual(tau_raw, 5.0, places=0)
        self.assertLess(tau_out, tau_raw)

    # ── 3. Torque clamping ──────────────────────────────────────

    def test_11_nominal_passthrough(self):
        """Nominal mode passes torque unchanged."""
        self._feed_all(n=30, tau=5.0)
        self.assertIsNotNone(self.last_torque_out)
        self.assertAlmostEqual(self.last_torque_out, 5.0, places=1)

    def test_12_torque_limit_clamp(self):
        """Torque_limit clamps to override_tau_limit (2.0)."""
        self._request_mode('torque_limit')
        self._feed_all(n=30, tau=5.0)
        self.assertIsNotNone(self.last_torque_out)
        self.assertLessEqual(abs(self.last_torque_out), 2.1)

    def test_13_stop_zero_torque(self):
        """Stop mode (zero_torque_only) outputs 0."""
        self._request_mode('stop')
        self._feed_all(n=30, tau=5.0)
        self.assertIsNotNone(self.last_torque_out)
        self.assertAlmostEqual(self.last_torque_out, 0.0, places=1)

    # ── 4. Admittance freeze ────────────────────────────────────

    def test_14_freeze_on_stop(self):
        """Entering stop freezes admittance."""
        self._request_mode('stop')
        self._spin(0.1)
        self.assertTrue(self.last_freeze)

    def test_15_unfreeze_stop_to_nominal(self):
        """stop -> nominal unfreezes."""
        self._request_mode('stop')
        self._spin(0.1)
        self._request_mode('nominal')
        self._spin(0.1)
        self.assertFalse(self.last_freeze)

    def test_16_unfreeze_stop_to_compliant(self):
        """stop -> compliant unfreezes (FIX BUG 5)."""
        self._request_mode('stop')
        self._spin(0.1)
        self.assertTrue(self.last_freeze)
        self._request_mode('compliant')
        self._spin(0.1)
        self.assertFalse(self.last_freeze)

    def test_17_unfreeze_stop_to_torque_limit(self):
        """stop -> torque_limit unfreezes (FIX BUG 5)."""
        self._request_mode('stop')
        self._spin(0.1)
        self._request_mode('torque_limit')
        self._spin(0.1)
        self.assertFalse(self.last_freeze)

    # ── 5. Watchdog ─────────────────────────────────────────────

    def test_18_all_alive_no_safe_stop(self):
        """All channels alive — no safe_stop triggered."""
        self.safe_stop_received = False
        for _ in range(50):
            self._pub_all()
            self._spin(0.01)
        self.assertFalse(self.safe_stop_received)

    def test_19_channel_death_safe_stop(self):
        """Dead torque channel triggers /safe_stop_request."""
        self.safe_stop_received = False
        for _ in range(100):
            self._pub_all()
            self._spin(0.05)

        from std_msgs.msg import Float64, Float64MultiArray
        from sensor_msgs.msg import JointState
        for _ in range(30):
            js = JointState()
            js.header.stamp = self.node.get_clock().now().to_msg()
            js.name = ['rev_crank']
            js.position = [0.5]
            js.velocity = [0.1]
            js.effort = [0.0]
            self.pub_js.publish(js)

            traj = Float64MultiArray()
            traj.data = [0.5, 0.1, 0.0]
            self.pub_traj.publish(traj)

            ext = Float64()
            ext.data = 0.0
            self.pub_ext.publish(ext)

            self._spin(0.05)

        self.assertTrue(
            self.safe_stop_received,
            'Watchdog should call /safe_stop_request on channel death')


@launch_testing.post_shutdown_test()
class TestShutdown(unittest.TestCase):
    """Verify clean shutdown."""

    def test_exit_code(self, proc_info):
        launch_testing.asserts.assertExitCodes(
            proc_info, allowable_exit_codes=[0, 1, -2, -15])