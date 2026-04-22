#!/usr/bin/env python3
"""
ROS2 node for controlled fault injection on the exoskeleton digital twin
(plant + admittance controller + trajectory controller).

Working principle
-----------------
The node acts as a proxy on one of four critical topics in the architecture.
It subscribes to the original topic, optionally corrupts the message, and
republishes on a "_faulted" topic. The rest of the architecture is remapped
to use the faulted topic via the ROS2 launch file (see remapping section below).

Channels
--------
  CHANNEL 0 — /exo_dynamics/tau_ext_theta  (Float64)
    Simulates a fault in the estimated user force projected onto theta.
    Example: degraded wrist force sensor, bias in the wrench estimate.

  CHANNEL 1 — /trajectory_ref  (Float64MultiArray: [theta_ref, theta_dot_ref, theta_ddot_ref])
    Simulates a fault in the outer loop (stuck admittance controller,
    corrupted reference, oscillation in the position/velocity reference).

  CHANNEL 2 — /torque  (Float64)
    Simulates a fault in the inner loop (trajectory controller).
    Corresponds to the scenario "error in the velocity/position control of a finger":
    the actuated torque does not match the commanded torque.

  CHANNEL 3 — /joint_states  (sensor_msgs/JointState)
    Simulates a fault in the position sensor (encoder) in the feedback path.
    The fault is applied to theta (position) and/or theta_dot (velocity) of the
    joint 'rev_crank', leaving all other joints in the message untouched.
    Example: encoder drift, noise on position signal, frozen reading.

Fault modes
-----------
  none    — transparent pass-through, no signal alteration
  offset  — adds a constant bias to the signal
  noise   — adds continuous Gaussian noise
  freeze  — completely blocks publication of the faulted topic
  scale   — multiplies the signal by a factor (e.g. 0.0 → zero torque)
  spike   — injects a single impulse of amplitude fault_magnitude, then reverts

Faults can be activated/deactivated at runtime without restarting the node:
    ros2 param set /fault_injector fault_active true
    ros2 param set /fault_injector fault_type offset
    ros2 param set /fault_injector fault_magnitude 2.0

Remapping schema
----------------
The node subscribes to the original topic and publishes on a "_faulted" topic.
To insert it into the architecture without modifying other nodes, use ROS2
topic remapping in the launch file:

    # In the launch file:
    # 1. fault_injector subscribes to /torque (original from trajectory_controller)
    # 2. fault_injector publishes to /torque_faulted
    # 3. the plant is launched with remapping /torque → /torque_faulted

Topics
------
  CHANNEL 0:
    Sub:  /exo_dynamics/tau_ext_theta         exoskeletron_safety_msgs/Float64Stamped
    Pub:  /exo_dynamics/tau_ext_theta_faulted exoskeletron_safety_msgs/Float64Stamped

  CHANNEL 1:
    Sub:  /trajectory_ref                     exoskeletron_safety_msgs/Float64ArrayStamped
    Pub:  /trajectory_ref_faulted             exoskeletron_safety_msgs/Float64ArrayStamped

  CHANNEL 2:
    Sub:  /torque                             exoskeletron_safety_msgs/Float64Stamped
    Pub:  /torque_faulted                     exoskeletron_safety_msgs/Float64Stamped

  CHANNEL 3:
    Sub:  /joint_states                       sensor_msgs/JointState
    Pub:  /joint_states_faulted               sensor_msgs/JointState

  Always:
    Pub:  /fault_injector/status              exoskeletron_safety_msgs/Float64ArrayStamped
          Layout: [channel, fault_type_id, fault_active, fault_magnitude,
                   n_injections, last_raw, last_faulted, delta]

ROS2 Parameters
---------------
  channel           (int,   default 2)        channel to inject fault on
                                              0=tau_ext_theta, 1=trajectory_ref,
                                              2=torque, 3=joint_states
  fault_active      (bool,  default False)    enable/disable fault injection
  fault_type        (str,   default 'offset') fault type: none/offset/noise/freeze/scale/spike
  fault_magnitude   (float, default 1.0)      fault amplitude
  noise_std         (float, default 0.1)      Gaussian noise std dev (fault_type=noise only)
  target_index      (int,   default 0)        Float64MultiArray element to corrupt (channel 1 only)
                                              0=theta_ref, 1=theta_dot_ref, 2=theta_ddot_ref
  fault_js_field    (str,   default 'position') JointState field to corrupt (channel 3 only)
                                              'position' → theta, 'velocity' → theta_dot, 'both'
  joint_name        (str,   default 'rev_crank') joint name to corrupt (channel 3 only)
  publish_rate      (float, default 200.0)    status topic rate [Hz]
  spike_duration    (float, default 0.05)     spike duration [s] (fault_type=spike only)

Usage example
-------------
  # Terminal 1: start node
  ros2 run <pkg> fault_injector --ros-args -p channel:=2 -p fault_type:=offset -p fault_magnitude:=3.0

  # Terminal 2: activate at runtime
  ros2 param set /fault_injector fault_active true

  # Terminal 3: change type at runtime
  ros2 param set /fault_injector fault_type freeze

  # Terminal 4: deactivate
  ros2 param set /fault_injector fault_active false

  # Monitor status
  ros2 topic echo /fault_injector/status
"""

import time
import numpy as np
import rclpy

from rclpy.node import Node
from exoskeletron_safety_msgs.msg import Float64Stamped, Float64ArrayStamped
from sensor_msgs.msg import JointState


# ============================================================
# Fault type string → numeric ID (used in the status topic)
# ============================================================
FAULT_TYPE_MAP = {
    'none':   0,
    'offset': 1,
    'noise':  2,
    'freeze': 3,
    'scale':  4,
    'spike':  5,
}

# ============================================================
# Channel metadata
# ============================================================
CHANNEL_INFO = {
    0: {
        'name': 'tau_ext_theta',
        'sub_topic': '/exo_dynamics/tau_ext_theta',
        'pub_topic': '/exo_dynamics/tau_ext_theta_faulted',
        'msg_type': 'Float64Stamped',
        'description': 'User force projected onto theta (force sensor)',
    },
    1: {
        'name': 'trajectory_ref',
        'sub_topic': '/trajectory_ref',
        'pub_topic': '/trajectory_ref_faulted',
        'msg_type': 'Float64ArrayStamped',
        'description': 'Trajectory reference [theta_ref, theta_dot_ref, theta_ddot_ref]',
    },
    2: {
        'name': 'torque',
        'sub_topic': '/torque',
        'pub_topic': '/torque_faulted',
        'msg_type': 'Float64Stamped',
        'description': 'Actuated torque (inner loop / finger)',
    },
    3: {
        'name': 'joint_states',
        'sub_topic': '/joint_states',
        'pub_topic': '/joint_states_faulted',
        'msg_type': 'JointState',
        'description': 'Position/velocity encoder sensor (feedback path)',
    },
}


class FaultInjector(Node):

    def __init__(self):
        super().__init__('fault_injector')

        # ============================================================
        # PARAMETERS
        # ============================================================
        self.declare_parameter('channel',         2)
        self.declare_parameter('fault_active',    False)
        self.declare_parameter('fault_type',      'offset')
        self.declare_parameter('fault_magnitude', 1.0)
        self.declare_parameter('noise_std',       0.1)
        self.declare_parameter('target_index',    0)
        self.declare_parameter('publish_rate',    200.0)
        self.declare_parameter('spike_duration',  0.05)
        self.declare_parameter('fault_js_field',  'position')
        self.declare_parameter('joint_name',      'rev_crank')

        channel = int(self.get_parameter('channel').value)
        publish_rate = float(self.get_parameter('publish_rate').value)

        if channel not in CHANNEL_INFO:
            self.get_logger().error(
                f"Invalid channel {channel}. Choose from {list(CHANNEL_INFO.keys())}."
            )
            raise ValueError(f"Invalid channel: {channel}")

        self._channel = channel
        self._ch_info = CHANNEL_INFO[channel]

        # ============================================================
        # INTERNAL STATE
        # ============================================================
        self._frozen_value = None     # first value seen when freeze was activated
        self._freeze_logged = False   # prevents repeated log messages during freeze
        self._n_injections = 0
        self._last_raw = 0.0
        self._last_faulted = 0.0

        self._spike_active = False
        self._spike_start_time = None

        # ============================================================
        # ROS I/O — subscriber and publisher depend on the channel type
        # ============================================================
        if self._ch_info['msg_type'] == 'Float64Stamped':
            self._sub = self.create_subscription(
                Float64Stamped,
                self._ch_info['sub_topic'],
                self._cb_float64,
                10
            )
            self._pub = self.create_publisher(
                Float64Stamped,
                self._ch_info['pub_topic'],
                10
            )

        elif self._ch_info['msg_type'] == 'JointState':
            self._sub = self.create_subscription(
                JointState,
                self._ch_info['sub_topic'],
                self._cb_joint_states,
                10
            )
            self._pub = self.create_publisher(
                JointState,
                self._ch_info['pub_topic'],
                10
            )

        else:  # Float64ArrayStamped
            self._sub = self.create_subscription(
                Float64ArrayStamped,
                self._ch_info['sub_topic'],
                self._cb_multiarray,
                10
            )
            self._pub = self.create_publisher(
                Float64ArrayStamped,
                self._ch_info['pub_topic'],
                10
            )

        self._pub_status = self.create_publisher(
            Float64ArrayStamped,
            '/fault_injector/status',
            10
        )

        self._timer_status = self.create_timer(
            1.0 / publish_rate,
            self._publish_status
        )

        self.get_logger().info(
            f"FaultInjector started\n"
            f"  Channel         : {channel} — {self._ch_info['description']}\n"
            f"  Sub topic       : {self._ch_info['sub_topic']}\n"
            f"  Pub topic       : {self._ch_info['pub_topic']}\n"
            f"  fault_active    : {self.get_parameter('fault_active').value}\n"
            f"  fault_type      : {self.get_parameter('fault_type').value}\n"
            f"  fault_magnitude : {self.get_parameter('fault_magnitude').value}\n"
            f"\nTo activate at runtime:\n"
            f"  ros2 param set /fault_injector fault_active true\n"
            f"  ros2 param set /fault_injector fault_type freeze\n"
            f"  ros2 param set /fault_injector fault_magnitude 2.0"
        )

    # ============================================================
    # UTILITIES
    # ============================================================

    def _current_fault_config(self):
        fault_active = bool(self.get_parameter('fault_active').value)
        fault_type = str(self.get_parameter('fault_type').value).lower().strip()
        fault_magnitude = float(self.get_parameter('fault_magnitude').value)
        noise_std = float(self.get_parameter('noise_std').value)
        spike_duration = float(self.get_parameter('spike_duration').value)
        return fault_active, fault_type, fault_magnitude, noise_std, spike_duration

    def _is_hard_freeze_active(self) -> bool:
        """
        Hard freeze: the node keeps receiving messages but publishes NOTHING
        on the faulted topic, simulating a dead publisher.
        """
        fault_active, fault_type, _, _, _ = self._current_fault_config()
        return fault_active and fault_type == 'freeze'

    def _reset_transient_states_if_needed(self):
        """
        Reset internal freeze/spike state when those fault types are no longer active.
        Called at the top of every callback so that switching fault_type at runtime
        cleanly resets the previous transient state.
        """
        fault_active, fault_type, _, _, _ = self._current_fault_config()

        if (not fault_active) or (fault_type != 'freeze'):
            self._frozen_value = None
            self._freeze_logged = False

        if (not fault_active) or (fault_type != 'spike'):
            self._spike_active = False
            self._spike_start_time = None

    # ============================================================
    # CALLBACKS — message reception
    # ============================================================

    def _cb_float64(self, msg: Float64Stamped):
        """
        Callback for Float64Stamped channels (tau_ext_theta, torque).
        If hard freeze is active, nothing is published on the faulted topic.
        """
        self._reset_transient_states_if_needed()

        raw_val = float(msg.data)
        self._last_raw = raw_val

        if self._is_hard_freeze_active():
            if self._frozen_value is None:
                self._frozen_value = raw_val
                self._last_faulted = raw_val

            if not self._freeze_logged:
                self.get_logger().info(
                    f"[FREEZE] Publishing blocked on {self._ch_info['pub_topic']}. "
                    f"Last observed value: {raw_val:.6f}"
                )
                self._freeze_logged = True

            self._n_injections += 1
            return

        faulted_val = self._apply_fault_scalar(raw_val)
        self._last_faulted = faulted_val

        out = Float64Stamped()
        out.header.stamp = self.get_clock().now().to_msg()
        out.data = faulted_val
        self._pub.publish(out)

    def _cb_multiarray(self, msg: Float64ArrayStamped):
        """
        Callback for Float64ArrayStamped channel (trajectory_ref).
        If hard freeze is active, nothing is published on the faulted topic.
        """
        self._reset_transient_states_if_needed()

        data = list(msg.data)
        if not data:
            if not self._is_hard_freeze_active():
                self._pub.publish(msg)
            return

        target_idx = int(self.get_parameter('target_index').value)
        target_idx = max(0, min(target_idx, len(data) - 1))

        raw_val = float(data[target_idx])
        self._last_raw = raw_val

        if self._is_hard_freeze_active():
            if self._frozen_value is None:
                self._frozen_value = raw_val
                self._last_faulted = raw_val

            if not self._freeze_logged:
                self.get_logger().info(
                    f"[FREEZE] Publishing blocked on {self._ch_info['pub_topic']}. "
                    f"Last observed value: {raw_val:.6f}"
                )
                self._freeze_logged = True

            self._n_injections += 1
            return

        faulted_val = self._apply_fault_scalar(raw_val)
        self._last_faulted = faulted_val

        data[target_idx] = faulted_val

        out = Float64ArrayStamped()
        out.header.stamp = self.get_clock().now().to_msg()
        out.data = data
        self._pub.publish(out)

    def _cb_joint_states(self, msg: JointState):
        """
        Callback for channel 3 — JointState.
        If hard freeze is active, nothing is published on the faulted topic.
        """
        self._reset_transient_states_if_needed()

        joint_name = str(self.get_parameter('joint_name').value)
        js_field = str(self.get_parameter('fault_js_field').value).lower().strip()

        try:
            idx = list(msg.name).index(joint_name)
        except ValueError:
            if not self._is_hard_freeze_active():
                self._pub.publish(msg)
            return

        candidate_raw = None
        if js_field in ('position', 'both') and idx < len(msg.position):
            candidate_raw = float(msg.position[idx])
        elif js_field in ('velocity', 'both') and idx < len(msg.velocity):
            candidate_raw = float(msg.velocity[idx])

        if candidate_raw is not None:
            self._last_raw = candidate_raw

        if self._is_hard_freeze_active():
            if self._frozen_value is None and candidate_raw is not None:
                self._frozen_value = candidate_raw
                self._last_faulted = candidate_raw

            if not self._freeze_logged:
                self.get_logger().info(
                    f"[FREEZE] Publishing blocked on {self._ch_info['pub_topic']} "
                    f"for joint '{joint_name}'."
                )
                self._freeze_logged = True

            self._n_injections += 1
            return

        out = JointState()
        out.header = msg.header
        out.name = list(msg.name)
        out.position = list(msg.position)
        out.velocity = list(msg.velocity)
        out.effort = list(msg.effort)

        if js_field in ('position', 'both') and idx < len(out.position):
            raw_pos = float(out.position[idx])
            self._last_raw = raw_pos
            faulted_pos = self._apply_fault_scalar(raw_pos)
            self._last_faulted = faulted_pos
            out.position[idx] = faulted_pos

        if js_field in ('velocity', 'both') and idx < len(out.velocity):
            raw_vel = float(out.velocity[idx])

            if js_field == 'velocity':
                self._last_raw = raw_vel
                faulted_vel = self._apply_fault_scalar(raw_vel)
                self._last_faulted = faulted_vel
            else:
                faulted_vel = self._apply_fault_scalar(raw_vel)

            out.velocity[idx] = faulted_vel

        self._pub.publish(out)

    # ============================================================
    # FAULT LOGIC
    # ============================================================

    def _apply_fault_scalar(self, value: float) -> float:
        """
        Apply the configured fault to a scalar input value.
        Returns the (possibly corrupted) output value.
        Note: the 'freeze' case is handled in the callbacks as a full publication
        block, so it should never reach this method.
        """
        fault_active, fault_type, fault_magnitude, noise_std, spike_duration = self._current_fault_config()

        if not fault_active or fault_type == 'none':
            return value

        if fault_type == 'offset':
            self._n_injections += 1
            return value + fault_magnitude

        elif fault_type == 'noise':
            noise = float(np.random.normal(0.0, noise_std))
            self._n_injections += 1
            return value + noise

        elif fault_type == 'scale':
            self._n_injections += 1
            return value * fault_magnitude

        elif fault_type == 'spike':
            now = time.monotonic()

            if not self._spike_active:
                self._spike_active = True
                self._spike_start_time = now
                self.get_logger().info(
                    f"[SPIKE] Impulse injected: {fault_magnitude:.4f}, "
                    f"duration {spike_duration:.3f} s"
                )

            elapsed = now - self._spike_start_time
            if elapsed <= spike_duration:
                self._n_injections += 1
                return value + fault_magnitude
            else:
                return value

        elif fault_type == 'freeze':
            # Should not be reached: freeze is handled entirely in the callbacks.
            return value

        else:
            self.get_logger().warn(
                f"Unrecognised fault_type '{fault_type}'. "
                f"Valid values: none, offset, noise, freeze, scale, spike"
            )
            return value

    # ============================================================
    # STATUS PUBLICATION
    # ============================================================

    def _publish_status(self):
        """
        Publish fault injector state on /fault_injector/status.

        Float64ArrayStamped layout:
          [0]  channel
          [1]  fault_type_id
          [2]  fault_active (1.0 / 0.0)
          [3]  fault_magnitude
          [4]  n_injections
          [5]  last_raw
          [6]  last_faulted
          [7]  delta = last_faulted - last_raw
        """
        fault_active, fault_type, fault_magnitude, _, _ = self._current_fault_config()
        fault_type_id = float(FAULT_TYPE_MAP.get(fault_type, -1))

        delta = self._last_faulted - self._last_raw

        msg = Float64ArrayStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.data = [
            float(self._channel),
            fault_type_id,
            1.0 if fault_active else 0.0,
            fault_magnitude,
            float(self._n_injections),
            float(self._last_raw),
            float(self._last_faulted),
            float(delta),
        ]
        self._pub_status.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = FaultInjector()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
