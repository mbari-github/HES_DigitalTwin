#!/usr/bin/env python3

import time
import json
import numpy as np
import rclpy

from collections import deque
from rclpy.node import Node
from exoskeleton_safety_msgs.msg import Float64Stamped, Float64ArrayStamped
from sensor_msgs.msg import JointState


FAULT_TYPE_MAP = {
    'none': 0,
    'offset': 1,
    'noise': 2,
    'freeze': 3,
    'scale': 4,
    'spike': 5,
    'drift_linear': 6,
    'drift_parabolic': 7,
    'bias_drift': 8,
    'quantization': 9,
    'deadzone': 10,
    'saturation': 11,
    'dropout': 12,
    'delay': 13,
}


CHANNEL_INFO = {
    0: {'sub_topic': '/exo_dynamics/tau_ext_theta', 'pub_topic': '/exo_dynamics/tau_ext_theta_faulted', 'msg_type': 'Float64Stamped'},
    1: {'sub_topic': '/trajectory_ref', 'pub_topic': '/trajectory_ref_faulted', 'msg_type': 'Float64ArrayStamped'},
    2: {'sub_topic': '/torque', 'pub_topic': '/torque_faulted', 'msg_type': 'Float64Stamped'},
    3: {'sub_topic': '/joint_states', 'pub_topic': '/joint_states_faulted', 'msg_type': 'JointState'},
}


class FaultInjector(Node):

    def __init__(self):
        super().__init__('fault_injector')

        # ================= PARAMETERS =================
        self.declare_parameter('channel', 2)
        self.declare_parameter('fault_active', False)

        # pipeline configurabile
        self.declare_parameter('fault_pipeline_config', '[]')

        # legacy fallback
        self.declare_parameter('fault_type', 'offset')
        self.declare_parameter('fault_magnitude', 1.0)
        self.declare_parameter('noise_std', 0.1)

        self.declare_parameter('target_index', 0)
        self.declare_parameter('publish_rate', 200.0)
        self.declare_parameter('spike_duration', 0.05)
        self.declare_parameter('fault_js_field', 'position')
        self.declare_parameter('joint_name', 'rev_crank')

        self._channel = int(self.get_parameter('channel').value)
        self._ch_info = CHANNEL_INFO[self._channel]

        # ================= STATE =================
        self._n_injections = 0
        self._last_raw = 0.0
        self._last_faulted = 0.0

        self._frozen_value = None
        self._freeze_logged = False

        self._fault_state = {
            'bias_drift': {'bias': 0.0},
            'drift': {'step': 0},
            'delay': {'buffer': deque(maxlen=10000)},
            'spike': {'active': False, 'start_time': None},
        }

        # ================= ROS I/O =================
        if self._ch_info['msg_type'] == 'Float64Stamped':
            self._sub = self.create_subscription(Float64Stamped, self._ch_info['sub_topic'], self._cb_float64, 10)
            self._pub = self.create_publisher(Float64Stamped, self._ch_info['pub_topic'], 10)

        elif self._ch_info['msg_type'] == 'JointState':
            self._sub = self.create_subscription(JointState, self._ch_info['sub_topic'], self._cb_joint_states, 10)
            self._pub = self.create_publisher(JointState, self._ch_info['pub_topic'], 10)

        else:
            self._sub = self.create_subscription(Float64ArrayStamped, self._ch_info['sub_topic'], self._cb_multiarray, 10)
            self._pub = self.create_publisher(Float64ArrayStamped, self._ch_info['pub_topic'], 10)

        self._pub_status = self.create_publisher(Float64ArrayStamped, '/fault_injector/status', 10)

        rate = float(self.get_parameter('publish_rate').value)
        self.create_timer(1.0 / rate, self._publish_status)

    # ============================================================
    # PARSE PIPELINE CONFIG
    # ============================================================

    def _get_pipeline(self):
        cfg_str = self.get_parameter('fault_pipeline_config').value

        try:
            pipeline = json.loads(cfg_str)
            if isinstance(pipeline, list):
                return pipeline
        except Exception:
            pass

        # fallback legacy
        return [{
            "type": self.get_parameter('fault_type').value,
            "magnitude": self.get_parameter('fault_magnitude').value,
            "noise_std": self.get_parameter('noise_std').value
        }]

    # ============================================================
    # FREEZE (HARD)
    # ============================================================

    def _is_hard_freeze_active(self, pipeline):
        if not self.get_parameter('fault_active').value:
            return False

        for f in pipeline:
            if f.get("type", "") == "freeze":
                return True

        return False

    # ============================================================
    # PIPELINE
    # ============================================================

    def _apply_pipeline(self, value, pipeline):

        out = value

        for f in pipeline:
            out = self._apply_fault(f, out)

        return out

    # ============================================================
    # SINGLE FAULT (PARAMETRICO)
    # ============================================================

    def _apply_fault(self, cfg, value):

        ftype = cfg.get("type", "none")

        if ftype == 'none':
            return value

        if ftype == 'offset':
            return value + cfg.get("magnitude", 0.0)

        if ftype == 'scale':
            return value * cfg.get("factor", 1.0)

        if ftype == 'noise':
            return value + np.random.normal(0.0, cfg.get("std", 0.1))

        # ---- DRIFT ----
        if ftype == 'drift_linear':
            s = self._fault_state['drift']
            s['step'] += 1
            return value + cfg.get("slope", 0.0) * s['step']

        if ftype == 'drift_parabolic':
            s = self._fault_state['drift']
            s['step'] += 1
            return value + cfg.get("coeff", 0.0) * (s['step'] ** 2)

        if ftype == 'bias_drift':
            s = self._fault_state['bias_drift']
            s['bias'] += np.random.normal(0.0, cfg.get("std", 0.001))
            return value + s['bias']

        # ---- SENSOR ----
        if ftype == 'quantization':
            res = cfg.get("resolution", 0.01)
            return res * round(value / res)

        if ftype == 'deadzone':
            eps = cfg.get("threshold", 0.01)
            return 0.0 if abs(value) < eps else value

        if ftype == 'saturation':
            lim = cfg.get("limit", 1.0)
            return max(-lim, min(lim, value))

        if ftype == 'dropout':
            if np.random.rand() < cfg.get("prob", 0.1):
                return self._last_faulted
            return value

        if ftype == 'delay':
            buf = self._fault_state['delay']['buffer']
            d = int(cfg.get("steps", 1))

            buf.append(value)
            if len(buf) > d:
                return buf[0]
            return value

        # ---- SPIKE ----
        if ftype == 'spike':
            s = self._fault_state['spike']
            now = time.monotonic()

            if not s['active']:
                s['active'] = True
                s['start_time'] = now

            if now - s['start_time'] <= cfg.get("duration", 0.05):
                return value + cfg.get("magnitude", 1.0)

        return value

    # ============================================================
    # CALLBACKS
    # ============================================================

    def _cb_float64(self, msg):
        raw = float(msg.data)
        self._last_raw = raw

        pipeline = self._get_pipeline()

        if self._is_hard_freeze_active(pipeline):
            if self._frozen_value is None:
                self._frozen_value = raw
                self._last_faulted = raw
            return

        val = self._apply_pipeline(raw, pipeline)
        self._last_faulted = val

        out = Float64Stamped()
        out.header.stamp = self.get_clock().now().to_msg()
        out.data = val
        self._pub.publish(out)

    def _cb_multiarray(self, msg):
        data = list(msg.data)
        idx = int(self.get_parameter('target_index').value)

        raw = float(data[idx])
        self._last_raw = raw

        pipeline = self._get_pipeline()

        if self._is_hard_freeze_active(pipeline):
            if self._frozen_value is None:
                self._frozen_value = raw
                self._last_faulted = raw
            return

        data[idx] = self._apply_pipeline(raw, pipeline)
        self._last_faulted = data[idx]

        out = Float64ArrayStamped()
        out.header.stamp = self.get_clock().now().to_msg()
        out.data = data
        self._pub.publish(out)

    def _cb_joint_states(self, msg):
        joint = self.get_parameter('joint_name').value

        try:
            idx = list(msg.name).index(joint)
        except ValueError:
            self._pub.publish(msg)
            return

        raw = msg.position[idx]
        self._last_raw = raw

        pipeline = self._get_pipeline()

        if self._is_hard_freeze_active(pipeline):
            if self._frozen_value is None:
                self._frozen_value = raw
                self._last_faulted = raw
            return

        out = JointState()
        out.header = msg.header
        out.name = list(msg.name)
        out.position = list(msg.position)
        out.velocity = list(msg.velocity)

        out.position[idx] = self._apply_pipeline(raw, pipeline)
        self._last_faulted = out.position[idx]

        self._pub.publish(out)

    # ============================================================
    # STATUS
    # ============================================================

    def _publish_status(self):
        delta = self._last_faulted - self._last_raw

        msg = Float64ArrayStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.data = [
            float(self._channel),
            0.0,
            1.0 if self.get_parameter('fault_active').value else 0.0,
            0.0,
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