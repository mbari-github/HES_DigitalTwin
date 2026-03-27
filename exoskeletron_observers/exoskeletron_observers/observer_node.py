#!/usr/bin/env python3

import math
from collections import deque

import rclpy
from rclpy.node import Node

from std_msgs.msg import Float64, Float64MultiArray
from sensor_msgs.msg import JointState


def ema(prev: float, new: float, alpha: float) -> float:
    return alpha * new + (1.0 - alpha) * prev


class ObserverNode(Node):

    def __init__(self):
        super().__init__('observer_node')

        # ── Parametri ─────────────────────────────────────────────
        self.declare_parameter('joint_name', 'rev_crank')
        self.declare_parameter('publish_rate', 200.0)

        self.declare_parameter('fric_visc', 2.0)
        self.declare_parameter('fric_coul', 2.0)
        self.declare_parameter('fric_eps', 0.005)
        self.declare_parameter('damping_theta', 1.0)

        self.declare_parameter('obs1_pole_1', -15.0)
        self.declare_parameter('obs1_pole_2', -20.0)

        # NEW — Admittance params
        self.declare_parameter('adm_M_virt', 1.0)
        self.declare_parameter('adm_D_virt', 10.0)
        self.declare_parameter('adm_K_virt', 50.0)
        self.declare_parameter('adm_theta_eq', 0.0)

        self.declare_parameter('residual_filter_alpha', 0.05)
        self.declare_parameter('rms_window', 50)

        self._joint_name = self.get_parameter('joint_name').value
        rate = self.get_parameter('publish_rate').value
        self._dt = 1.0 / rate

        # ── Input ─────────────────────────────────────────────
        self._theta_meas = 0.0
        self._theta_dot_meas = 0.0
        self._tau_ext_meas = 0.0

        self._tau_m = 0.0
        self._M_eff = 1.0
        self._proj = 0.0
        self._tau_pass = 0.0

        # trajectory ref (NEW)
        self._theta_ref = 0.0
        self._theta_dot_ref = 0.0
        self._theta_ddot_ref = 0.0

        # flags
        self._js_received = False
        self._ff_received = False
        self._tau_ext_received = False
        self._tau_m_received = False
        self._traj_received = False

        # ── Observer 1 ────────────────────────────────────────
        self._theta_hat = 0.0
        self._theta_dot_hat = 0.0
        self._obs1_initialized = False

        # ── Observer 2 (Admittance) ───────────────────────────
        self._tau_ext_hat = 0.0

        # ── Residui ───────────────────────────────────────────
        self._state_res_filtered = 0.0
        self._torque_res_filtered = 0.0

        win = int(self.get_parameter('rms_window').value)
        self._state_res_buf = deque(maxlen=win)
        self._torque_res_buf = deque(maxlen=win)

        # ── Subscribers ───────────────────────────────────────
        self.create_subscription(JointState, '/joint_states', self._js_cb, 10)
        self.create_subscription(Float64, '/exo_dynamics/tau_ext_theta', self._tau_ext_cb, 10)
        self.create_subscription(Float64MultiArray, '/exo_dynamics/ff_terms', self._ff_cb, 10)
        self.create_subscription(Float64, '/torque_raw', self._tau_m_cb, 10)

        # NEW
        self.create_subscription(Float64MultiArray, '/trajectory_ref', self._traj_cb, 10)

        # ── Publishers ────────────────────────────────────────
        self._pub_state = self.create_publisher(Float64, '/observer/state_residual', 10)
        self._pub_torque = self.create_publisher(Float64, '/observer/torque_residual', 10)
        self._pub_debug = self.create_publisher(Float64MultiArray, '/observer/debug', 10)

        self.create_timer(self._dt, self._update)

    # ─────────────────────────────────────────────────────────
    # Callbacks
    # ─────────────────────────────────────────────────────────

    def _js_cb(self, msg):
        try:
            idx = msg.name.index(self._joint_name)
        except ValueError:
            return

        self._theta_meas = msg.position[idx]
        self._theta_dot_meas = msg.velocity[idx]
        self._js_received = True

    def _tau_ext_cb(self, msg):
        self._tau_ext_meas = msg.data
        self._tau_ext_received = True

    def _tau_m_cb(self, msg):
        self._tau_m = msg.data
        self._tau_m_received = True

    def _ff_cb(self, msg):
        d = msg.data
        if len(d) > 0:
            self._M_eff = max(d[0], 1e-6)
        if len(d) > 1:
            self._proj = d[1]
        if len(d) > 3:
            self._tau_pass = d[3]
        self._ff_received = True

    def _traj_cb(self, msg):
        d = msg.data
        if len(d) < 3:
            return
        self._theta_ref = d[0]
        self._theta_dot_ref = d[1]
        self._theta_ddot_ref = d[2]
        self._traj_received = True

    # ─────────────────────────────────────────────────────────
    # Loop
    # ─────────────────────────────────────────────────────────

    def _update(self):

        if not (self._js_received and self._ff_received and
                self._tau_m_received and self._traj_received):
            return

        dt = self._dt

        # ── PARAMETRI ────────────────────────────────────────
        fric_visc = self.get_parameter('fric_visc').value
        damping = self.get_parameter('damping_theta').value
        alpha = self.get_parameter('residual_filter_alpha').value

        # ─────────────────────────────────────────────────────
        # OBSERVER 1 (Luenberger)
        # ─────────────────────────────────────────────────────

        p1 = self.get_parameter('obs1_pole_1').value
        p2 = self.get_parameter('obs1_pole_2').value

        D = fric_visc + damping
        D_over_M = D / self._M_eff

        l1 = -(p1 + p2) - D_over_M
        l2 = p1 * p2 - l1 * D_over_M

        if not self._obs1_initialized:
            self._theta_hat = self._theta_meas
            self._theta_dot_hat = self._theta_dot_meas
            self._obs1_initialized = True

        tau_fric = D * self._theta_dot_hat

        theta_ddot_model = (
            self._tau_m + self._tau_pass - self._proj - tau_fric
        ) / self._M_eff

        innov = self._theta_meas - self._theta_hat
        state_res = innov

        self._theta_dot_hat += dt * (theta_ddot_model + l2 * innov)
        self._theta_hat += dt * (self._theta_dot_hat + l1 * innov)

        # ─────────────────────────────────────────────────────
        # OBSERVER 2 — Admittance Inversion (NUOVO)
        # ─────────────────────────────────────────────────────

        Mv = self.get_parameter('adm_M_virt').value
        Dv = self.get_parameter('adm_D_virt').value
        Kv = self.get_parameter('adm_K_virt').value
        theta_eq = self.get_parameter('adm_theta_eq').value

        self._tau_ext_hat = (
            Mv * self._theta_ddot_ref +
            Dv * self._theta_dot_ref +
            Kv * (self._theta_ref - theta_eq)
        )

        torque_res = self._tau_ext_meas - self._tau_ext_hat

        # ── FILTRI ───────────────────────────────────────────
        self._state_res_filtered = ema(self._state_res_filtered, state_res, alpha)
        self._torque_res_filtered = ema(self._torque_res_filtered, torque_res, alpha)

        self._state_res_buf.append(state_res**2)
        self._torque_res_buf.append(torque_res**2)

        # ── PUB ──────────────────────────────────────────────
        msg = Float64()
        msg.data = state_res
        self._pub_state.publish(msg)

        msg = Float64()
        msg.data = torque_res
        self._pub_torque.publish(msg)

        dbg = Float64MultiArray()
        dbg.data = [
            self._theta_hat,
            self._theta_dot_hat,
            state_res,
            self._tau_ext_hat,
            torque_res,
            self._state_res_filtered,
            self._torque_res_filtered,
            self._theta_meas,
            self._theta_dot_meas,
            self._tau_m,
            self._tau_ext_meas,
            self._theta_ref,
            self._theta_dot_ref,
            self._theta_ddot_ref,
        ]
        self._pub_debug.publish(dbg)


def main():
    rclpy.init()
    node = ObserverNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()