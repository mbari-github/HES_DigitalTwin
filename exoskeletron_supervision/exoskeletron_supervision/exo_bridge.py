import copy
from typing import Optional

import rclpy
from rclpy.node import Node

from std_msgs.msg import Float64, Float64MultiArray
from std_srvs.srv import SetBool
from sensor_msgs.msg import JointState


class ExoBridge(Node):
    def __init__(self):
        super().__init__('exo_bridge')

        # ---------------------------------------------------------
        # Parameters
        # ---------------------------------------------------------
        self.declare_parameter('publish_rate_hz', 200.0)

        self.declare_parameter('override_tau_limit', 1.0)

        self.declare_parameter('compliant_tau_limit', 0.6)
        self.declare_parameter('compliant_vel_limit', 1.0)
        self.declare_parameter('compliant_acc_limit', 2.0)

        self.declare_parameter('stop_mode', 'hold_only')
        # stop_mode:
        # - hold_only
        # - hold_and_zero_torque
        # - zero_torque_only

        self.publish_rate_hz = float(self.get_parameter('publish_rate_hz').value)

        self.override_tau_limit = float(self.get_parameter('override_tau_limit').value)

        self.compliant_tau_limit = float(self.get_parameter('compliant_tau_limit').value)
        self.compliant_vel_limit = float(self.get_parameter('compliant_vel_limit').value)
        self.compliant_acc_limit = float(self.get_parameter('compliant_acc_limit').value)

        self.stop_mode = str(self.get_parameter('stop_mode').value)

        # ---------------------------------------------------------
        # Internal state
        # ---------------------------------------------------------
        self.control_mode = 'nominal'
        # allowed:
        # - nominal
        # - torque_limit
        # - compliant
        # - stop

        self.last_joint_state: Optional[JointState] = None
        self.last_trajectory_ref_in: Optional[Float64MultiArray] = None
        self.last_torque_in: Optional[Float64] = None
        self.last_tau_ext: Optional[Float64] = None

        self.theta_hold: Optional[float] = None

        # ---------------------------------------------------------
        # Subscribers
        # ---------------------------------------------------------
        self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10
        )

        self.create_subscription(
            Float64MultiArray,
            '/trajectory_ref_raw',
            self.trajectory_ref_callback,
            10
        )

        self.create_subscription(
            Float64,
            '/torque_raw',
            self.torque_callback,
            10
        )

        self.create_subscription(
            Float64,
            '/exo_dynamics/tau_ext_theta',
            self.tau_ext_callback,
            10
        )

        # ---------------------------------------------------------
        # Publishers
        # ---------------------------------------------------------
        self.trajectory_ref_pub = self.create_publisher(
            Float64MultiArray,
            '/trajectory_ref',
            10
        )

        self.torque_pub = self.create_publisher(
            Float64,
            '/torque',
            10
        )

        self.bridge_status_pub = self.create_publisher(
            Float64MultiArray,
            '/exo_bridge/status',
            10
        )

        # ---------------------------------------------------------
        # Services
        # ---------------------------------------------------------
        self.create_service(
            SetBool,
            '/stop_robot',
            self.stop_robot_callback
        )

        self.create_service(
            SetBool,
            '/override_control',
            self.override_control_callback
        )

        self.create_service(
            SetBool,
            '/compliant_control',
            self.compliant_control_callback
        )

        # ---------------------------------------------------------
        # Timer
        # ---------------------------------------------------------
        timer_period = 1.0 / self.publish_rate_hz
        self.timer = self.create_timer(timer_period, self.control_loop)

        self.get_logger().info('ExoBridge started')
        self.get_logger().info(f'publish_rate_hz = {self.publish_rate_hz}')
        self.get_logger().info(f'override_tau_limit = {self.override_tau_limit}')
        self.get_logger().info(f'compliant_tau_limit = {self.compliant_tau_limit}')
        self.get_logger().info(f'compliant_vel_limit = {self.compliant_vel_limit}')
        self.get_logger().info(f'compliant_acc_limit = {self.compliant_acc_limit}')
        self.get_logger().info(f'stop_mode = {self.stop_mode}')

    # ------------------------------------------------------------------
    # Subscriber callbacks
    # ------------------------------------------------------------------
    def joint_state_callback(self, msg: JointState):
        self.last_joint_state = msg

    def trajectory_ref_callback(self, msg: Float64MultiArray):
        self.last_trajectory_ref_in = msg

    def torque_callback(self, msg: Float64):
        self.last_torque_in = msg

    def tau_ext_callback(self, msg: Float64):
        self.last_tau_ext = msg

    # ------------------------------------------------------------------
    # Service callbacks
    # ------------------------------------------------------------------
    def stop_robot_callback(self, request: SetBool.Request, response: SetBool.Response):
        if request.data:
            self.control_mode = 'stop'
            current_theta = self.get_current_theta()
            if current_theta is not None:
                self.theta_hold = current_theta
                self.get_logger().warn(
                    f'STOP activated. Holding rev_crank at theta = {self.theta_hold:.6f}'
                )
            else:
                self.theta_hold = None
                self.get_logger().warn(
                    'STOP activated, but rev_crank position is unavailable.'
                )
            response.message = 'Stop activated.'
        else:
            self.theta_hold = None
            self.control_mode = 'nominal'
            self.get_logger().info('STOP deactivated. Returning to nominal mode.')
            response.message = 'Stop deactivated.'

        response.success = True
        return response

    def override_control_callback(self, request: SetBool.Request, response: SetBool.Response):
        if request.data:
            self.control_mode = 'torque_limit'
            self.get_logger().warn('TORQUE_LIMIT override activated.')
            response.message = 'Torque limit override activated.'
        else:
            self.control_mode = 'nominal'
            self.get_logger().info('TORQUE_LIMIT override deactivated.')
            response.message = 'Torque limit override deactivated.'

        response.success = True
        return response

    def compliant_control_callback(self, request: SetBool.Request, response: SetBool.Response):
        if request.data:
            self.control_mode = 'compliant'
            self.get_logger().warn('COMPLIANT control activated.')
            response.message = 'Compliant control activated.'
        else:
            self.control_mode = 'nominal'
            self.get_logger().info('COMPLIANT control deactivated.')
            response.message = 'Compliant control deactivated.'

        response.success = True
        return response

    # ------------------------------------------------------------------
    # Main loop
    # ------------------------------------------------------------------
    def control_loop(self):
        safe_ref = self.compute_safe_trajectory_ref()
        safe_tau = self.compute_safe_torque()

        if safe_ref is not None:
            self.trajectory_ref_pub.publish(safe_ref)

        if safe_tau is not None:
            self.torque_pub.publish(safe_tau)

        self.publish_status()

    # ------------------------------------------------------------------
    # Safe trajectory generation
    # ------------------------------------------------------------------
    def compute_safe_trajectory_ref(self) -> Optional[Float64MultiArray]:
        if self.control_mode == 'stop':
            return self.build_hold_reference()

        if self.last_trajectory_ref_in is None:
            return None

        safe_ref = copy.deepcopy(self.last_trajectory_ref_in)

        if len(safe_ref.data) < 3:
            self.get_logger().warn('Received malformed trajectory_ref_raw')
            return safe_ref

        theta_ref = float(safe_ref.data[0])
        theta_dot_ref = float(safe_ref.data[1])
        theta_ddot_ref = float(safe_ref.data[2])

        if self.control_mode == 'compliant':
            theta_dot_ref = self.clamp(
                theta_dot_ref,
                -self.compliant_vel_limit,
                self.compliant_vel_limit
            )
            theta_ddot_ref = self.clamp(
                theta_ddot_ref,
                -self.compliant_acc_limit,
                self.compliant_acc_limit
            )

        safe_ref.data = [theta_ref, theta_dot_ref, theta_ddot_ref]
        return safe_ref

    # ------------------------------------------------------------------
    # Safe torque generation
    # ------------------------------------------------------------------
    def compute_safe_torque(self) -> Optional[Float64]:
        if self.last_torque_in is None:
            return None

        tau_in = float(self.last_torque_in.data)
        msg = Float64()

        if self.control_mode == 'stop':
            msg.data = self.compute_stop_torque_value(tau_in)
            return msg

        if self.control_mode == 'torque_limit':
            msg.data = self.clamp(
                tau_in,
                -self.override_tau_limit,
                self.override_tau_limit
            )
            return msg

        if self.control_mode == 'compliant':
            msg.data = self.clamp(
                tau_in,
                -self.compliant_tau_limit,
                self.compliant_tau_limit
            )
            return msg

        msg.data = tau_in
        return msg

    def compute_stop_torque_value(self, tau_in: float) -> float:
        if self.stop_mode == 'zero_torque_only':
            return 0.0

        if self.stop_mode == 'hold_only':
            return tau_in

        # default: hold_and_zero_torque
        return 0.0

    def build_hold_reference(self) -> Optional[Float64MultiArray]:
        theta = self.theta_hold

        if theta is None:
            theta = self.get_current_theta()

        if theta is None:
            if self.last_trajectory_ref_in is not None:
                return copy.deepcopy(self.last_trajectory_ref_in)
            return None

        msg = Float64MultiArray()
        msg.data = [float(theta), 0.0, 0.0]
        return msg

    # ------------------------------------------------------------------
    # Helpers
    # ------------------------------------------------------------------
    def get_rev_crank_index(self) -> Optional[int]:
        if self.last_joint_state is None:
            return None

        try:
            return self.last_joint_state.name.index('rev_crank')
        except ValueError:
            return None

    def get_current_theta(self) -> Optional[float]:
        idx = self.get_rev_crank_index()
        if idx is None:
            return None
        if len(self.last_joint_state.position) <= idx:
            return None
        return float(self.last_joint_state.position[idx])

    def get_current_theta_dot(self) -> Optional[float]:
        idx = self.get_rev_crank_index()
        if idx is None:
            return None
        if len(self.last_joint_state.velocity) <= idx:
            return None
        return float(self.last_joint_state.velocity[idx])

    @staticmethod
    def clamp(value: float, vmin: float, vmax: float) -> float:
        return max(vmin, min(value, vmax))

    def control_mode_to_id(self) -> float:
        if self.control_mode == 'nominal':
            return 0.0
        if self.control_mode == 'torque_limit':
            return 1.0
        if self.control_mode == 'compliant':
            return 2.0
        if self.control_mode == 'stop':
            return 3.0
        return -1.0

    def publish_status(self):
        msg = Float64MultiArray()

        theta = self.get_current_theta()
        theta_dot = self.get_current_theta_dot()
        tau_in = self.last_torque_in.data if self.last_torque_in is not None else float('nan')
        tau_ext = self.last_tau_ext.data if self.last_tau_ext is not None else float('nan')
        theta_hold = self.theta_hold if self.theta_hold is not None else float('nan')

        stop_active = 1.0 if self.control_mode == 'stop' else 0.0
        override_active = 1.0 if self.control_mode in ('torque_limit', 'compliant') else 0.0

        # Layout:
        # [0] stop_active
        # [1] override_active
        # [2] theta
        # [3] theta_dot
        # [4] theta_hold
        # [5] tau_in
        # [6] tau_ext
        # [7] control_mode_id
        msg.data = [
            stop_active,
            override_active,
            theta if theta is not None else float('nan'),
            theta_dot if theta_dot is not None else float('nan'),
            theta_hold,
            tau_in,
            tau_ext,
            self.control_mode_to_id()
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