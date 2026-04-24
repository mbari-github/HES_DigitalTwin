#!/usr/bin/env python3
"""

Canali supportati (identici a fault_injector.py):
  0 → /exo_dynamics/tau_ext_theta      (Float64Stamped)
  1 → /trajectory_ref                  (Float64ArrayStamped)
  2 → /torque                          (Float64Stamped)
  3 → /joint_states                    (JointState)

Fault types disponibili:
  none          — passthrough trasparente
  offset        — bias costante additivo
  scale         — moltiplicazione per fattore
  noise         — rumore Gaussiano additivo
  freeze        — blocca la pubblicazione (simula nodo morto)
  spike         — impulso singolo di durata configurabile
  drift_linear  — deriva lineare nel tempo
  drift_parabolic — deriva parabolica nel tempo
  bias_drift    — random walk del bias (Gaussiano incrementale)
  quantization  — quantizzazione a risoluzione fissa
  deadzone      — zona morta: segnale → 0 se |val| < threshold
  saturation    — saturazione simmetrica a ±limit
  dropout       — campioni casuali sostituiti con l'ultimo valore noto
  delay         — ritardo di N campioni (buffer FIFO)

Configurazione pipeline (YAML):
  fault_pipeline_config: >
    [
      {"type": "bias_drift", "std": 0.001},
      {"type": "noise", "std": 0.01},
      {"type": "quantization", "resolution": 0.005}
    ]

Se la pipeline è vuota o non valida, fallback sui parametri legacy:
  fault_type, fault_magnitude, noise_std

ROS 2 Parameters:
  channel           (int,   default 2)
  fault_active      (bool,  default False)
  fault_pipeline_config (str, default '[]')
  fault_type        (str,   default 'offset')   legacy
  fault_magnitude   (float, default 1.0)        legacy
  noise_std         (float, default 0.1)        legacy
  target_index      (int,   default 0)          per Float64ArrayStamped
  joint_name        (str,   default 'rev_crank') per JointState
  fault_js_field    (str,   default 'position') per JointState
  publish_rate      (float, default 200.0)
  spike_duration    (float, default 0.05)       legacy spike
"""

import time
import json
import numpy as np
import rclpy

from collections import deque
from rclpy.node import Node
from exoskeleton_safety_msgs.msg import Float64Stamped, Float64ArrayStamped
from sensor_msgs.msg import JointState


# ── Fault type string → numeric ID (usato nello status topic) ──
FAULT_TYPE_MAP = {
    'none':             0,
    'offset':           1,
    'noise':            2,
    'freeze':           3,
    'scale':            4,
    'spike':            5,
    'drift_linear':     6,
    'drift_parabolic':  7,
    'bias_drift':       8,
    'quantization':     9,
    'deadzone':        10,
    'saturation':      11,
    'dropout':         12,
    'delay':           13,
}

# ── Channel metadata ──
CHANNEL_INFO = {
    0: {
        'sub_topic': '/exo_dynamics/tau_ext_theta',
        'pub_topic': '/exo_dynamics/tau_ext_theta_faulted',
        'msg_type': 'Float64Stamped',
        'description': 'User force projected onto theta (force sensor)',
    },
    1: {
        'sub_topic': '/trajectory_ref',
        'pub_topic': '/trajectory_ref_faulted',
        'msg_type': 'Float64ArrayStamped',
        'description': 'Trajectory reference [theta_ref, theta_dot_ref, theta_ddot_ref]',
    },
    2: {
        'sub_topic': '/torque',
        'pub_topic': '/torque_faulted',
        'msg_type': 'Float64Stamped',
        'description': 'Actuated torque (inner loop)',
    },
    3: {
        'sub_topic': '/joint_states',
        'pub_topic': '/joint_states_faulted',
        'msg_type': 'JointState',
        'description': 'Position/velocity encoder (feedback path)',
    },
}


class FaultFramework(Node):

    def __init__(self):
        super().__init__('fault_injector')

        # ── Parameters ──────────────────────────────────────────────
        self.declare_parameter('channel',               2)
        self.declare_parameter('fault_active',          False)
        self.declare_parameter('fault_pipeline_config', '[]')

        # Legacy single-fault parameters (fallback)
        self.declare_parameter('fault_type',            'offset')
        self.declare_parameter('fault_magnitude',       1.0)
        self.declare_parameter('noise_std',             0.1)
        self.declare_parameter('spike_duration',        0.05)

        # Channel-specific
        self.declare_parameter('target_index',          0)
        self.declare_parameter('joint_name',            'rev_crank')
        self.declare_parameter('fault_js_field',        'position')

        self.declare_parameter('publish_rate',          200.0)

        self._channel = int(self.get_parameter('channel').value)
        if self._channel not in CHANNEL_INFO:
            self.get_logger().error(
                f'Invalid channel {self._channel}. Choose from {list(CHANNEL_INFO.keys())}.'
            )
            raise ValueError(f'Invalid channel: {self._channel}')

        self._ch_info = CHANNEL_INFO[self._channel]
        publish_rate = float(self.get_parameter('publish_rate').value)

        # ── Internal state ──────────────────────────────────────────

        # Diagnostics
        self._n_injections = 0
        self._last_raw = 0.0
        self._last_faulted = 0.0

        # Freeze state
        self._frozen_value = None
        self._freeze_logged = False

        # Per-fault-type transient states
        self._fault_state = {
            'bias_drift': {'bias': 0.0},
            'drift':      {'step': 0},
            'delay':      {'buffer': deque(maxlen=10000)},
            'spike':      {'active': False, 'start_time': None},
        }

        # Tracks the last pipeline config string to detect runtime changes
        self._last_pipeline_str = ''

        # ── ROS I/O ─────────────────────────────────────────────────
        if self._ch_info['msg_type'] == 'Float64Stamped':
            self._sub = self.create_subscription(
                Float64Stamped,
                self._ch_info['sub_topic'],
                self._cb_float64,
                10,
            )
            self._pub = self.create_publisher(
                Float64Stamped,
                self._ch_info['pub_topic'],
                10,
            )

        elif self._ch_info['msg_type'] == 'JointState':
            self._sub = self.create_subscription(
                JointState,
                self._ch_info['sub_topic'],
                self._cb_joint_states,
                10,
            )
            self._pub = self.create_publisher(
                JointState,
                self._ch_info['pub_topic'],
                10,
            )

        else:  # Float64ArrayStamped
            self._sub = self.create_subscription(
                Float64ArrayStamped,
                self._ch_info['sub_topic'],
                self._cb_multiarray,
                10,
            )
            self._pub = self.create_publisher(
                Float64ArrayStamped,
                self._ch_info['pub_topic'],
                10,
            )

        self._pub_status = self.create_publisher(
            Float64ArrayStamped,
            '/fault_injector/status',
            10,
        )

        self._timer_status = self.create_timer(
            1.0 / publish_rate,
            self._publish_status,
        )

        self.get_logger().info(
            f'FaultFramework started\n'
            f'  Channel         : {self._channel} — {self._ch_info["description"]}\n'
            f'  Sub topic       : {self._ch_info["sub_topic"]}\n'
            f'  Pub topic       : {self._ch_info["pub_topic"]}\n'
            f'  fault_active    : {self.get_parameter("fault_active").value}\n'
            f'\nTo activate at runtime:\n'
            f'  ros2 param set /fault_injector fault_active true\n'
            f'  ros2 param set /fault_injector fault_pipeline_config '
            f'\'[{{"type":"offset","magnitude":1.0}}]\''
        )

    # ════════════════════════════════════════════════════════════════
    # PIPELINE PARSING
    # ════════════════════════════════════════════════════════════════

    def _get_pipeline(self):
        """
        Parses fault_pipeline_config JSON string into a list of dicts.
        Falls back to the legacy single-fault parameters if the JSON
        is empty or invalid.
        """
        cfg_str = str(self.get_parameter('fault_pipeline_config').value)

        # Detect runtime pipeline change and reset transient states
        if cfg_str != self._last_pipeline_str:
            self._reset_transient_states()
            self._last_pipeline_str = cfg_str

        try:
            pipeline = json.loads(cfg_str)
            if isinstance(pipeline, list) and len(pipeline) > 0:
                return pipeline
        except Exception:
            pass

        # Legacy fallback: single fault from individual parameters
        return [{
            'type':       str(self.get_parameter('fault_type').value),
            'magnitude':  float(self.get_parameter('fault_magnitude').value),
            'noise_std':  float(self.get_parameter('noise_std').value),
            'duration':   float(self.get_parameter('spike_duration').value),
        }]

    def _reset_transient_states(self):
        """
        Resets all per-fault-type transient states.
        Called automatically when the pipeline configuration changes at runtime.
        """
        self._fault_state['bias_drift']['bias'] = 0.0
        self._fault_state['drift']['step'] = 0
        self._fault_state['delay']['buffer'].clear()
        self._fault_state['spike']['active'] = False
        self._fault_state['spike']['start_time'] = None
        self._frozen_value = None
        self._freeze_logged = False
        self.get_logger().info(
            'FaultFramework: pipeline changed — transient states reset.'
        )

    # ════════════════════════════════════════════════════════════════
    # FREEZE CHECK
    # ════════════════════════════════════════════════════════════════

    def _is_hard_freeze_active(self, pipeline):
        """
        Returns True if fault_active=True AND at least one stage in the
        pipeline is a 'freeze' type.
        """
        if not bool(self.get_parameter('fault_active').value):
            return False
        return any(f.get('type', '') == 'freeze' for f in pipeline)

    # ════════════════════════════════════════════════════════════════
    # PIPELINE APPLICATION
    # ════════════════════════════════════════════════════════════════

    def _apply_pipeline(self, value, pipeline):
        """
        Applies each fault stage in the pipeline sequentially.
        The output of one stage is the input of the next.
        """
        out = value
        for stage in pipeline:
            out = self._apply_single_fault(stage, out)
        return out

    def _apply_single_fault(self, cfg, value):
        """
        Applies one fault stage to a scalar value.
        Returns the (possibly corrupted) output.
        Note: 'freeze' is handled at the callback level (full publication
        suppression), so it is a no-op here.
        """
        ftype = cfg.get('type', 'none')

        if ftype == 'none':
            return value

        # ── Simple additive / multiplicative ────────────────────────
        if ftype == 'offset':
            self._n_injections += 1
            return value + float(cfg.get('magnitude', 0.0))

        if ftype == 'scale':
            self._n_injections += 1
            return value * float(cfg.get('factor', 1.0))

        if ftype == 'noise':
            self._n_injections += 1
            return value + float(np.random.normal(0.0, cfg.get('std', 0.1)))

        # ── Drift ────────────────────────────────────────────────────
        if ftype == 'drift_linear':
            s = self._fault_state['drift']
            s['step'] += 1
            self._n_injections += 1
            return value + float(cfg.get('slope', 0.0)) * s['step']

        if ftype == 'drift_parabolic':
            s = self._fault_state['drift']
            s['step'] += 1
            self._n_injections += 1
            return value + float(cfg.get('coeff', 0.0)) * (s['step'] ** 2)

        if ftype == 'bias_drift':
            s = self._fault_state['bias_drift']
            s['bias'] += float(np.random.normal(0.0, cfg.get('std', 0.001)))
            self._n_injections += 1
            return value + s['bias']

        # ── Sensor degradation ───────────────────────────────────────
        if ftype == 'quantization':
            res = float(cfg.get('resolution', 0.01))
            if res <= 0.0:
                return value
            self._n_injections += 1
            return res * round(value / res)

        if ftype == 'deadzone':
            eps = float(cfg.get('threshold', 0.01))
            self._n_injections += 1
            return 0.0 if abs(value) < eps else value

        if ftype == 'saturation':
            lim = float(cfg.get('limit', 1.0))
            self._n_injections += 1
            return float(np.clip(value, -lim, lim))

        if ftype == 'dropout':
            if float(np.random.rand()) < float(cfg.get('prob', 0.1)):
                self._n_injections += 1
                return self._last_faulted   # hold last known value
            return value

        if ftype == 'delay':
            buf = self._fault_state['delay']['buffer']
            d = max(1, int(cfg.get('steps', 1)))
            buf.append(value)
            if len(buf) > d:
                self._n_injections += 1
                return float(buf[0])
            return value

        # ── Spike ────────────────────────────────────────────────────
        if ftype == 'spike':
            s = self._fault_state['spike']
            now = time.monotonic()

            if not s['active']:
                s['active'] = True
                s['start_time'] = now
                self.get_logger().info(
                    f'FaultFramework [SPIKE] amplitude={cfg.get("magnitude", 1.0):.4f} '
                    f'duration={cfg.get("duration", 0.05):.3f}s'
                )

            elapsed = now - s['start_time']
            if elapsed <= float(cfg.get('duration', 0.05)):
                self._n_injections += 1
                return value + float(cfg.get('magnitude', 1.0))
            else:
                # Spike expired — reset so the next activation can fire again
                # if fault is toggled off and on.
                s['active'] = False
                s['start_time'] = None
                return value

        # ── Freeze (handled at callback level, no-op here) ───────────
        if ftype == 'freeze':
            return value

        self.get_logger().warn(
            f'FaultFramework: unknown fault type "{ftype}" — skipping stage.'
        )
        return value

    # ════════════════════════════════════════════════════════════════
    # CALLBACKS
    # ════════════════════════════════════════════════════════════════

    def _cb_float64(self, msg: Float64Stamped):
        raw = float(msg.data)
        self._last_raw = raw

        # Passthrough when fault is disabled
        if not bool(self.get_parameter('fault_active').value):
            out = Float64Stamped()
            out.header.stamp = self.get_clock().now().to_msg()
            out.data = raw
            self._last_faulted = raw
            self._pub.publish(out)
            return

        pipeline = self._get_pipeline()

        # Hard freeze: suppress publication entirely
        if self._is_hard_freeze_active(pipeline):
            if self._frozen_value is None:
                self._frozen_value = raw
                self._last_faulted = raw
            if not self._freeze_logged:
                self.get_logger().info(
                    f'FaultFramework [FREEZE] blocking {self._ch_info["pub_topic"]} | '
                    f'last_raw={raw:.6f}'
                )
                self._freeze_logged = True
            self._n_injections += 1
            return

        faulted = self._apply_pipeline(raw, pipeline)
        self._last_faulted = faulted

        out = Float64Stamped()
        out.header.stamp = self.get_clock().now().to_msg()
        out.data = faulted
        self._pub.publish(out)

    def _cb_multiarray(self, msg: Float64ArrayStamped):
        data = list(msg.data)
        if not data:
            if not bool(self.get_parameter('fault_active').value):
                self._pub.publish(msg)
            return

        target_idx = int(self.get_parameter('target_index').value)
        target_idx = max(0, min(target_idx, len(data) - 1))

        raw = float(data[target_idx])
        self._last_raw = raw

        # Passthrough when fault is disabled
        if not bool(self.get_parameter('fault_active').value):
            out = Float64ArrayStamped()
            out.header.stamp = self.get_clock().now().to_msg()
            out.data = data
            self._last_faulted = raw
            self._pub.publish(out)
            return

        pipeline = self._get_pipeline()

        if self._is_hard_freeze_active(pipeline):
            if self._frozen_value is None:
                self._frozen_value = raw
                self._last_faulted = raw
            if not self._freeze_logged:
                self.get_logger().info(
                    f'FaultFramework [FREEZE] blocking {self._ch_info["pub_topic"]} | '
                    f'last_raw={raw:.6f}'
                )
                self._freeze_logged = True
            self._n_injections += 1
            return

        faulted = self._apply_pipeline(raw, pipeline)
        self._last_faulted = faulted
        data[target_idx] = faulted

        out = Float64ArrayStamped()
        out.header.stamp = self.get_clock().now().to_msg()
        out.data = data
        self._pub.publish(out)

    def _cb_joint_states(self, msg: JointState):
        joint_name = str(self.get_parameter('joint_name').value)
        js_field = str(self.get_parameter('fault_js_field').value).lower().strip()

        try:
            idx = list(msg.name).index(joint_name)
        except ValueError:
            # Joint not found: forward unchanged
            self._pub.publish(msg)
            return

        # Extract the raw value for diagnostics
        candidate_raw = None
        if js_field in ('position', 'both') and idx < len(msg.position):
            candidate_raw = float(msg.position[idx])
        elif js_field == 'velocity' and idx < len(msg.velocity):
            candidate_raw = float(msg.velocity[idx])

        if candidate_raw is not None:
            self._last_raw = candidate_raw

        # Passthrough when fault is disabled
        if not bool(self.get_parameter('fault_active').value):
            out = JointState()
            out.header = msg.header
            out.name = list(msg.name)
            out.position = list(msg.position)
            out.velocity = list(msg.velocity)
            out.effort = list(msg.effort)
            if candidate_raw is not None:
                self._last_faulted = candidate_raw
            self._pub.publish(out)
            return

        pipeline = self._get_pipeline()

        if self._is_hard_freeze_active(pipeline):
            if self._frozen_value is None and candidate_raw is not None:
                self._frozen_value = candidate_raw
                self._last_faulted = candidate_raw
            if not self._freeze_logged:
                self.get_logger().info(
                    f'FaultFramework [FREEZE] blocking {self._ch_info["pub_topic"]} | '
                    f'joint={joint_name}'
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
            faulted_pos = self._apply_pipeline(raw_pos, pipeline)
            self._last_faulted = faulted_pos
            out.position[idx] = faulted_pos

        if js_field in ('velocity', 'both') and idx < len(out.velocity):
            raw_vel = float(out.velocity[idx])
            if js_field == 'velocity':
                self._last_raw = raw_vel
                faulted_vel = self._apply_pipeline(raw_vel, pipeline)
                self._last_faulted = faulted_vel
            else:
                # 'both': position already done above, apply independently
                faulted_vel = self._apply_pipeline(raw_vel, pipeline)
            out.velocity[idx] = faulted_vel

        self._pub.publish(out)

    # ════════════════════════════════════════════════════════════════
    # STATUS PUBLISHING
    # ════════════════════════════════════════════════════════════════

    def _publish_status(self):
        """
        Publishes diagnostic status on /fault_injector/status.

        Layout (Float64ArrayStamped):
          [0]  channel
          [1]  fault_type_id  (first stage type, or 0 if pipeline)
          [2]  fault_active   (1.0 / 0.0)
          [3]  fault_magnitude (first stage, or 0.0 if pipeline)
          [4]  n_injections
          [5]  last_raw
          [6]  last_faulted
          [7]  delta = last_faulted - last_raw
        """
        fault_active = bool(self.get_parameter('fault_active').value)
        pipeline = self._get_pipeline()

        # Extract info from the first pipeline stage for the status fields
        first_type = pipeline[0].get('type', 'none') if pipeline else 'none'
        fault_type_id = float(FAULT_TYPE_MAP.get(first_type, -1))
        first_magnitude = float(pipeline[0].get('magnitude', 0.0)) if pipeline else 0.0

        delta = self._last_faulted - self._last_raw

        msg = Float64ArrayStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.data = [
            float(self._channel),
            fault_type_id,
            1.0 if fault_active else 0.0,
            first_magnitude,
            float(self._n_injections),
            float(self._last_raw),
            float(self._last_faulted),
            float(delta),
        ]
        self._pub_status.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = FaultFramework()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()