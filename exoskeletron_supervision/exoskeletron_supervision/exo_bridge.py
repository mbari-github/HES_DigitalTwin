#!/usr/bin/env python3
"""
Safety gateway between the controllers and the plant.

Role
----
The bridge sits between the trajectory controller / admittance controller and
the dynamics node. Every control cycle it:
  1. Checks watchdogs on all critical topics.
  2. Optionally limits or replaces the trajectory reference and torque command
     according to the current safety mode.
  3. Publishes the processed commands to the plant.
  4. Publishes a status vector used by the safety state machine for mode
     coherence checking and automatic downgrade decisions.

Design notes
------------
- Freeze logic: when entering STOP mode the bridge publishes freeze=True on
  /admittance/freeze so the admittance controller holds its virtual state.
  On any transition OUT of STOP (not just to 'nominal') freeze is released.
  This prevents the admittance from being left frozen if the state machine
  later adds a stop→compliant path.

- tau_raw vs tau_out: the status topic publishes tau_raw (pre-clamp torque
  from the controller) at data[8] in addition to tau_out (post-clamp) at
  data[5]. The state machine uses tau_raw for the downgrade decision because
  tau_out is always within limits by construction and would never signal
  that the fault condition has cleared.

Layout of /exo_bridge/status (exoskeletron_safety_msgs/Float64ArrayStamped):
  data[0] = is_stop           (1.0 if mode==stop, else 0.0)
  data[1] = is_limited        (1.0 if mode==torque_limit or compliant)
  data[2] = theta             (rev_crank position)
  data[3] = theta_dot         (rev_crank velocity)
  data[4] = theta_hold        (hold position in stop mode, nan if not active)
  data[5] = tau_out           (post-clamp torque actually sent to plant)
  data[6] = tau_ext_theta     (projected external force)
  data[7] = mode_id           (0=nominal, 1=torque_limit, 2=compliant, 3=stop)
  data[8] = tau_raw           (pre-clamp torque from the controller)
"""
import copy
from typing import Optional

import rclpy
from rclpy.node import Node

from std_msgs.msg import String, Bool
from exoskeletron_safety_msgs.msg import Float64Stamped, Float64ArrayStamped
from std_srvs.srv import SetBool
from sensor_msgs.msg import JointState

from exoskeletron_safety_msgs.srv import SetMode


# Channel names used by the watchdog
CH_TORQUE  = 'torque_raw'
CH_TRAJ    = 'trajectory_ref_raw'
CH_TAU_EXT = 'tau_ext_theta'
CH_JS      = 'joint_states'
ALL_CHANNELS = [CH_TORQUE, CH_TRAJ, CH_TAU_EXT, CH_JS]


class ExoBridge(Node):
    def __init__(self):
        super().__init__('exo_bridge')

        self.declare_parameter('publish_rate_hz',     200.0)
        self.declare_parameter('override_tau_limit',    1.0)
        self.declare_parameter('compliant_tau_limit',   0.6)
        self.declare_parameter('compliant_vel_limit',   1.0)
        self.declare_parameter('compliant_acc_limit',   2.0)
        self.declare_parameter('stop_mode',         'zero_torque_only')
        self.declare_parameter('hold_tau_limit',        1.0)
        self.declare_parameter('watchdog_timeout_sec',  0.5)
        self.declare_parameter('watchdog_grace_sec',    2.0)

        self.publish_rate_hz     = float(self.get_parameter('publish_rate_hz').value)
        self.override_tau_limit  = float(self.get_parameter('override_tau_limit').value)
        self.compliant_tau_limit = float(self.get_parameter('compliant_tau_limit').value)
        self.compliant_vel_limit = float(self.get_parameter('compliant_vel_limit').value)
        self.compliant_acc_limit = float(self.get_parameter('compliant_acc_limit').value)
        self.stop_mode           = str(self.get_parameter('stop_mode').value)
        self.hold_tau_limit      = float(self.get_parameter('hold_tau_limit').value)
        self.watchdog_timeout    = float(self.get_parameter('watchdog_timeout_sec').value)
        self.watchdog_grace      = float(self.get_parameter('watchdog_grace_sec').value)

        self.control_mode: str            = 'nominal'
        self.theta_hold: Optional[float]  = None

        self.last_joint_state:       Optional[JointState]           = None
        self.last_trajectory_ref_in: Optional[Float64ArrayStamped] = None
        self.last_torque_in:         Optional[Float64Stamped]       = None
        self.last_tau_ext:           Optional[Float64Stamped]       = None

        # Torque actually sent to the plant in the last cycle (post-clamp)
        self.tau_out_last: float = 0.0

        self._last_rx: dict[str, Optional[float]] = {
            CH_TORQUE:  None,
            CH_TRAJ:    None,
            CH_TAU_EXT: None,
            CH_JS:      None,
        }
        self._dead_channels: set[str] = set()
        self._start_time: float = self.get_clock().now().nanoseconds * 1e-9

        # ── Subscribers ──────────────────────────────────────────────
        self.create_subscription(JointState,           '/joint_states',               self._cb_js,  10)
        self.create_subscription(Float64ArrayStamped, '/trajectory_ref_raw',         self._cb_ref, 10)
        self.create_subscription(Float64Stamped,      '/torque_raw',                 self._cb_tau, 10)
        self.create_subscription(Float64Stamped,      '/exo_dynamics/tau_ext_theta', self._cb_ext, 10)

        # ── Publishers ───────────────────────────────────────────────
        self.trajectory_ref_pub  = self.create_publisher(Float64ArrayStamped, '/trajectory_ref',    10)
        self.torque_pub          = self.create_publisher(Float64Stamped,      '/torque',             10)
        self.bridge_status_pub   = self.create_publisher(Float64ArrayStamped, '/exo_bridge/status',  10)
        self.mode_pub            = self.create_publisher(String,            '/exo_bridge/mode',    10)
        self.admittance_freeze_pub = self.create_publisher(Bool,            '/admittance/freeze',  10)

        # ── Services ──────────────────────────────────────────────────
        self.create_service(SetMode, '/bridge/set_mode', self._cb_set_mode)

        # ── Client to state machine for safe stop escalation ──────────
        self._safe_stop_client = self.create_client(SetBool, '/safe_stop_request')

        # ── Main timer ───────────────────────────────────────────────
        self.timer = self.create_timer(1.0 / self.publish_rate_hz, self._control_loop)

        self.get_logger().info(
            f'ExoBridge started | '
            f'stop_mode={self.stop_mode} | '
            f'hold_tau_limit={self.hold_tau_limit:.3f} Nm | '
            f'watchdog_timeout={self.watchdog_timeout:.2f}s | '
            f'grace={self.watchdog_grace:.2f}s'
        )

    # ── Subscriber callbacks ─────────────────────────────────────────

    def _cb_js(self, msg: JointState):
        self.last_joint_state = msg
        self._touch(CH_JS)

    def _cb_ref(self, msg: Float64ArrayStamped):
        self.last_trajectory_ref_in = msg
        self._touch(CH_TRAJ)

    def _cb_tau(self, msg: Float64Stamped):
        self.last_torque_in = msg
        self._touch(CH_TORQUE)

    def _cb_ext(self, msg: Float64Stamped):
        self.last_tau_ext = msg
        self._touch(CH_TAU_EXT)

    def _touch(self, channel: str):
        """Update the last-received timestamp for a channel and clear its dead flag."""
        now = self.get_clock().now().nanoseconds * 1e-9
        self._last_rx[channel] = now
        if channel in self._dead_channels:
            self._dead_channels.discard(channel)
            self.get_logger().info(f'ExoBridge: channel [{channel}] recovered')

    # ── SetMode service ──────────────────────────────────────────────

    def _cb_set_mode(self, request: SetMode.Request, response: SetMode.Response):
        mode = request.mode.strip().lower()
        allowed = ('nominal', 'compliant', 'torque_limit', 'stop')
        if mode not in allowed:
            response.success = False
            response.message = f"Unknown mode '{mode}'. Allowed: {allowed}"
            self.get_logger().error(response.message)
            return response

        prev = self.control_mode
        self.control_mode = mode

        if mode == 'stop':
            theta = self.get_current_theta()
            self.theta_hold = theta
            self.get_logger().warn(
                f'Bridge mode: STOP | stop_mode={self.stop_mode} | theta_hold='
                + (f'{theta:.6f}' if theta is not None else 'unavailable')
            )
            # Freeze the admittance integrator to prevent drift during stop
            freeze_msg = Bool()
            freeze_msg.data = True
            self.admittance_freeze_pub.publish(freeze_msg)

        elif prev == 'stop':
            # Release the admittance freeze on any transition OUT of STOP.
            # Releasing only on stop→nominal would leave the admittance frozen
            # for any other exit path added in the future (e.g. stop→compliant).
            self.theta_hold = None
            self.get_logger().info(f'Bridge mode: {mode.upper()} (was STOP) | unfreeze admittance')
            freeze_msg = Bool()
            freeze_msg.data = False
            self.admittance_freeze_pub.publish(freeze_msg)

        elif mode == 'nominal':
            self.theta_hold = None
            self.get_logger().info(f'Bridge mode: NOMINAL (was {prev})')

        else:
            self.get_logger().warn(f'Bridge mode: {mode.upper()} (was {prev})')

        response.success = True
        response.message = f'Mode set to {mode}'
        return response

    # ── Control loop ─────────────────────────────────────────────────

    def _control_loop(self):
        self._check_watchdogs()

        ref = self._safe_trajectory_ref()
        tau = self._safe_torque()
        if ref is not None: self.trajectory_ref_pub.publish(ref)
        if tau is not None:
            self.tau_out_last = tau.data
            self.torque_pub.publish(tau)
        self._publish_status()
        self._publish_mode()

    # ── Watchdog ─────────────────────────────────────────────────────

    def _check_watchdogs(self):
        """
        Monitor all critical channels. After the grace period, if any channel
        has not been received within watchdog_timeout_sec, trigger a safe stop.
        """
        now = self.get_clock().now().nanoseconds * 1e-9
        if (now - self._start_time) < self.watchdog_grace:
            return

        newly_dead = []
        for ch in ALL_CHANNELS:
            last = self._last_rx[ch]
            if last is None:
                if ch not in self._dead_channels:
                    newly_dead.append((ch, -1.0))
                    self._dead_channels.add(ch)
                continue
            age = now - last
            if age > self.watchdog_timeout:
                if ch not in self._dead_channels:
                    newly_dead.append((ch, age))
                    self._dead_channels.add(ch)

        if not newly_dead:
            return

        for ch, age in newly_dead:
            if age < 0:
                self.get_logger().error(
                    f'ExoBridge WATCHDOG: channel [{ch}] never received → SAFE_STOP'
                )
            else:
                self.get_logger().error(
                    f'ExoBridge WATCHDOG: channel [{ch}] dead | age={age:.3f}s → SAFE_STOP'
                )

        self._request_safe_stop()

    def _request_safe_stop(self):
        if not self._safe_stop_client.wait_for_service(timeout_sec=0.3):
            self.get_logger().error(
                'ExoBridge WATCHDOG: /safe_stop_request not available!'
            )
            return
        req = SetBool.Request()
        req.data = True
        future = self._safe_stop_client.call_async(req)
        future.add_done_callback(self._safe_stop_response_cb)

    def _safe_stop_response_cb(self, future):
        try:
            result = future.result()
            self.get_logger().info(
                f'ExoBridge WATCHDOG: safe_stop response | '
                f'success={result.success} msg={result.message}'
            )
        except Exception as e:
            self.get_logger().error(f'ExoBridge WATCHDOG: safe_stop call failed: {e}')

    # ── Safe trajectory ──────────────────────────────────────────────

    def _safe_trajectory_ref(self) -> Optional[Float64ArrayStamped]:
        if self.control_mode == 'stop':
            return self._hold_reference()
        if self.last_trajectory_ref_in is None:
            return None
        ref = copy.deepcopy(self.last_trajectory_ref_in)
        if len(ref.data) < 3:
            self.get_logger().warn('Malformed trajectory_ref_raw')
            return ref
        theta_ref, theta_dot_ref, theta_ddot_ref = (
            float(ref.data[0]), float(ref.data[1]), float(ref.data[2])
        )
        if self.control_mode == 'compliant':
            theta_dot_ref  = self.clamp(theta_dot_ref,  -self.compliant_vel_limit, self.compliant_vel_limit)
            theta_ddot_ref = self.clamp(theta_ddot_ref, -self.compliant_acc_limit, self.compliant_acc_limit)
        ref.header.stamp = self.get_clock().now().to_msg()
        ref.data = [theta_ref, theta_dot_ref, theta_ddot_ref]
        return ref

    # ── Safe torque ──────────────────────────────────────────────────

    def _safe_torque(self) -> Optional[Float64Stamped]:
        if self.last_torque_in is None:
            return None
        tau_in = float(self.last_torque_in.data)
        msg = Float64Stamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        if self.control_mode == 'stop':
            msg.data = self._stop_torque(tau_in)
        elif self.control_mode == 'torque_limit':
            msg.data = self.clamp(tau_in, -self.override_tau_limit, self.override_tau_limit)
        elif self.control_mode == 'compliant':
            msg.data = self.clamp(tau_in, -self.compliant_tau_limit, self.compliant_tau_limit)
        else:
            msg.data = tau_in
        return msg

    def _stop_torque(self, tau_in: float) -> float:
        if self.stop_mode == 'zero_torque_only':
            return 0.0
        if self.stop_mode == 'hold_only':
            return self.clamp(tau_in, -self.hold_tau_limit, self.hold_tau_limit)
        # hold_and_zero_torque or unrecognised value → safe default
        return 0.0

    def _hold_reference(self) -> Optional[Float64ArrayStamped]:
        theta = self.theta_hold if self.theta_hold is not None else self.get_current_theta()
        if theta is None:
            if self.last_trajectory_ref_in is None:
                return None
            ref = copy.deepcopy(self.last_trajectory_ref_in)
            ref.header.stamp = self.get_clock().now().to_msg()
            return ref
        msg = Float64ArrayStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.data = [float(theta), 0.0, 0.0]
        return msg

    # ── Helpers ──────────────────────────────────────────────────────

    def _rev_crank_idx(self) -> Optional[int]:
        if self.last_joint_state is None:
            return None
        try:
            return self.last_joint_state.name.index('rev_crank')
        except ValueError:
            return None

    def get_current_theta(self) -> Optional[float]:
        idx = self._rev_crank_idx()
        if idx is None or idx >= len(self.last_joint_state.position):
            return None
        return float(self.last_joint_state.position[idx])

    def get_current_theta_dot(self) -> Optional[float]:
        idx = self._rev_crank_idx()
        if idx is None or idx >= len(self.last_joint_state.velocity):
            return None
        return float(self.last_joint_state.velocity[idx])

    @staticmethod
    def clamp(v, lo, hi):
        return max(lo, min(v, hi))

    def _mode_id(self) -> float:
        return {'nominal': 0.0, 'torque_limit': 1.0,
                'compliant': 2.0, 'stop': 3.0}.get(self.control_mode, -1.0)

    def _publish_mode(self):
        msg = String()
        msg.data = self.control_mode
        self.mode_pub.publish(msg)

    def _publish_status(self):
        theta     = self.get_current_theta()
        theta_dot = self.get_current_theta_dot()

        # tau_raw is the pre-clamp torque from the controller.
        # The state machine uses it for downgrade decisions: it must evaluate
        # whether the fault condition has cleared by looking at what the controller
        # WANTS to apply, not what is actually reaching the plant (which is always
        # within limits by construction and would never indicate fault recovery).
        tau_raw = self.last_torque_in.data if self.last_torque_in else float('nan')

        msg = Float64ArrayStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.data = [
            1.0 if self.control_mode == 'stop' else 0.0,                     # [0] is_stop
            1.0 if self.control_mode in ('torque_limit', 'compliant') else 0.0, # [1] is_limited
            theta     if theta     is not None else float('nan'),             # [2] theta
            theta_dot if theta_dot is not None else float('nan'),             # [3] theta_dot
            self.theta_hold if self.theta_hold is not None else float('nan'), # [4] theta_hold
            self.tau_out_last,                                                 # [5] tau_out (post-clamp)
            self.last_tau_ext.data if self.last_tau_ext else float('nan'),    # [6] tau_ext_theta
            self._mode_id(),                                                   # [7] mode_id
            float(tau_raw),                                                    # [8] tau_raw (pre-clamp)
        ]
        self.bridge_status_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = ExoBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
