#!/usr/bin/env python3
"""
ROS2 node for model-based fault detection on the exoskeleton.

Implements four complementary detection layers:

  OBSERVER 1 — Luenberger state observer (θ, θ̇)
  ─────────────────────────────────────────────────
  Uses the reduced 1-DOF plant model to predict θ̂ and θ̇_hat, then corrects
  with Luenberger gains. The residual (innovation) is sensitive to encoder and
  actuator faults, but has a detection latency of ~1-1.5 s for faults on τ_ext
  (channel 0) because the response is mediated through the admittance → tracking chain.

  OBSERVER 2 — Admittance inversion
  ────────────────────────────────────
  Reconstructs τ̂_ext from the trajectory references by inverting the admittance
  controller model. Sensitive to faults in the admittance loop.
  Invalidated when the reference is saturated.

  OBSERVER 3 — Generalized momentum observer
  ─────────────────────────────────────────────
  Direct estimate of the external torque discrepancy without computing θ̈
  (noisy derivative). Works on the integral of the generalized momentum:

      p(t) = M_eff · θ̇

      σ̇ = τ_known + r_mom          σ(0) = p(0)
      r_mom = K_mom · (p - σ)

  where τ_known = τ_m + τ_pass + τ_ext_meas - proj - friction.
  At steady state: r_mom → (τ_ext_real - τ_ext_measured).

  With K_mom = 50 the time constant is ~20 ms: detection latency
  ~40-60 ms for a 2 Nm offset (vs ~1.5 s for the Luenberger).

  GUARD 4 — τ_ext rate-of-change
  ──────────────────────────────────
  Monitors |Δτ_ext / Δt|. A physical human force cannot produce instantaneous
  steps (arm inertia, tissue compliance, muscle bandwidth ≈ 5-10 Hz). A 2 Nm
  jump in 5 ms = 400 Nm/s is necessarily artificial. Detection latency: 1 sample
  (~5 ms). Blind to slow drifts.

Topics
------
  Sub:  /joint_states                    sensor_msgs/JointState
        /exo_dynamics/tau_ext_theta      std_msgs/Float64
        /exo_dynamics/ff_terms           std_msgs/Float64MultiArray
        /torque_raw                      std_msgs/Float64
        /trajectory_ref                  std_msgs/Float64MultiArray
  Pub:  /observer/state_residual         std_msgs/Float64
        /observer/torque_residual        std_msgs/Float64
        /observer/momentum_residual      std_msgs/Float64
        /observer/tau_ext_rate_alarm     std_msgs/Float64
        /observer/state_rms              std_msgs/Float64
        /observer/torque_rms             std_msgs/Float64
        /observer/debug                  std_msgs/Float64MultiArray  (21 fields)

ROS2 Parameters
---------------
  joint_name             (str)    joint to observe
  publish_rate           (float)  loop frequency [Hz]

  # Plant model (must match dynamics_params.yaml)
  fric_visc              (float)  viscous friction [Nm·s/rad]
  fric_coul              (float)  Coulomb friction [Nm]
  fric_eps               (float)  tanh Coulomb threshold [rad/s]
  damping_theta          (float)  additional damping [Nm·s/rad]

  # Observer 1 — Luenberger
  obs1_pole_1            (float)  pole 1 (negative, stable)
  obs1_pole_2            (float)  pole 2 (negative, stable)

  # Observer 2 — Admittance inversion
  adm_M_virt             (float)  admittance virtual mass
  adm_D_virt             (float)  admittance virtual damping
  adm_K_virt             (float)  admittance virtual stiffness
  adm_theta_eq           (float)  equilibrium position
  adm_force_deadband     (float)  admittance force deadband
  adm_theta_ref_min      (float)  lower θ_ref limit
  adm_theta_ref_max      (float)  upper θ_ref limit
  adm_theta_dot_max      (float)  θ̇_ref velocity limit

  # Observer 3 — Generalized momentum
  mom_obs_gain           (float)  gain K_mom [1/s] (default 50 → τ ≈ 20 ms)

  # Guard 4 — τ_ext rate-of-change
  tau_ext_max_rate       (float)  threshold |dτ_ext/dt| [Nm/s] (default 100)

  # Residual filters
  residual_filter_alpha  (float)  EMA coefficient
  rms_window             (int)    RMS window [samples]
  torque_res_max         (float)  torque residual clamp [Nm]
"""

import math
from collections import deque

import rclpy
from rclpy.node import Node

from std_msgs.msg import Float64, Float64MultiArray
from sensor_msgs.msg import JointState


# ─────────────────────────────────────────────────────────────
# Utility functions
# ─────────────────────────────────────────────────────────────

def ema(prev: float, new: float, alpha: float) -> float:
    """Exponential moving average."""
    return alpha * new + (1.0 - alpha) * prev


def rms_from_buf(buf) -> float:
    """Root-mean-square from a buffer of squared values."""
    if len(buf) == 0:
        return 0.0
    return math.sqrt(sum(buf) / len(buf))


class ObserverNode(Node):

    def __init__(self):
        super().__init__('observer_node')

        # ── Parameters ────────────────────────────────────────────────
        self.declare_parameter('joint_name', 'rev_crank')
        self.declare_parameter('publish_rate', 200.0)

        # Plant model (must match dynamics_params.yaml)
        self.declare_parameter('fric_visc', 2.0)
        self.declare_parameter('fric_coul', 2.0)
        self.declare_parameter('fric_eps', 0.005)
        self.declare_parameter('damping_theta', 1.0)

        # Observer 1 — Luenberger poles
        self.declare_parameter('obs1_pole_1', -15.0)
        self.declare_parameter('obs1_pole_2', -20.0)

        # Observer 2 — Admittance inversion parameters
        self.declare_parameter('adm_M_virt', 0.5)
        self.declare_parameter('adm_D_virt', 5.0)
        self.declare_parameter('adm_K_virt', 2.0)
        self.declare_parameter('adm_theta_eq', 0.0)
        self.declare_parameter('adm_force_deadband', 0.0)
        self.declare_parameter('adm_theta_ref_min', -2.0)
        self.declare_parameter('adm_theta_ref_max', 2.0)
        self.declare_parameter('adm_theta_dot_max', 5.0)

        # Observer 3 — Momentum observer gain
        # K_mom [1/s]: time constant τ ≈ 1/K_mom.
        #   K_mom = 50  → τ ≈ 20 ms  (fast detection)
        #   K_mom = 20  → τ ≈ 50 ms  (more filtered)
        #   K_mom = 100 → τ ≈ 10 ms  (very fast, more noise-sensitive)
        self.declare_parameter('mom_obs_gain', 50.0)

        # Guard 4 — τ_ext rate-of-change threshold [Nm/s]
        # Physiological reference:
        #   - Maximum human force rate: ~5-20 Nm/s
        #   - Fault offset 2 Nm in 5 ms → 400 Nm/s
        # 100 Nm/s gives ample margin for normal operation.
        self.declare_parameter('tau_ext_max_rate', 100.0)

        # Residual filters
        self.declare_parameter('residual_filter_alpha', 0.05)
        self.declare_parameter('rms_window', 50)
        self.declare_parameter('torque_res_max', 0.5)

        self._joint_name = self.get_parameter('joint_name').value
        rate = self.get_parameter('publish_rate').value
        self._dt = 1.0 / rate

        # ── Input signals ─────────────────────────────────────────────
        self._theta_meas = 0.0
        self._theta_dot_meas = 0.0
        self._tau_ext_meas = 0.0

        self._tau_m = 0.0
        self._M_eff = 1.0
        self._proj = 0.0
        self._tau_pass = 0.0

        # Trajectory reference
        self._theta_ref = 0.0
        self._theta_dot_ref = 0.0
        self._theta_ddot_ref = 0.0

        # Reception flags (guard against running before all topics are available)
        self._js_received = False
        self._ff_received = False
        self._tau_ext_received = False
        self._tau_m_received = False
        self._traj_received = False

        # ── Observer 1 (Luenberger) state ─────────────────────────────
        self._theta_hat = 0.0
        self._theta_dot_hat = 0.0
        self._obs1_initialized = False

        # ── Observer 2 (Admittance Inversion) state ───────────────────
        self._tau_ext_hat = 0.0
        self._obs2_valid = False

        # ── Observer 3 (Momentum) state ───────────────────────────────
        self._mom_sigma = 0.0       # integrator σ(t)
        self._r_momentum = 0.0      # residual r(t)
        self._M_eff_prev = 1.0      # M_eff from previous step
        self._mom_initialized = False

        # ── Guard 4 (τ_ext rate-of-change) state ─────────────────────
        self._tau_ext_prev = 0.0
        self._tau_ext_rate = 0.0
        self._rate_alarm = False
        self._rate_guard_initialized = False

        # ── Residual filters ──────────────────────────────────────────
        self._state_res_filtered = 0.0
        self._torque_res_filtered = 0.0

        win = int(self.get_parameter('rms_window').value)
        self._state_res_buf = deque(maxlen=win)
        self._torque_res_buf = deque(maxlen=win)

        # ── Subscribers ───────────────────────────────────────────────
        self.create_subscription(JointState, '/joint_states', self._js_cb, 10)
        self.create_subscription(Float64, '/exo_dynamics/tau_ext_theta', self._tau_ext_cb, 10)
        self.create_subscription(Float64MultiArray, '/exo_dynamics/ff_terms', self._ff_cb, 10)
        self.create_subscription(Float64, '/torque_raw', self._tau_m_cb, 10)
        self.create_subscription(Float64MultiArray, '/trajectory_ref', self._traj_cb, 10)

        # ── Publishers ────────────────────────────────────────────────
        self._pub_state = self.create_publisher(Float64, '/observer/state_residual', 10)
        self._pub_torque = self.create_publisher(Float64, '/observer/torque_residual', 10)
        self._pub_momentum = self.create_publisher(Float64, '/observer/momentum_residual', 10)
        self._pub_rate_alarm = self.create_publisher(Float64, '/observer/tau_ext_rate_alarm', 10)
        self._pub_state_rms = self.create_publisher(Float64, '/observer/state_rms', 10)
        self._pub_torque_rms = self.create_publisher(Float64, '/observer/torque_rms', 10)
        self._pub_debug = self.create_publisher(Float64MultiArray, '/observer/debug', 10)

        self.create_timer(self._dt, self._update)

        self.get_logger().info(
            f"ObserverNode started | rate={rate:.0f} Hz | "
            f"poles=[{self.get_parameter('obs1_pole_1').value}, "
            f"{self.get_parameter('obs1_pole_2').value}] | "
            f"K_mom={self.get_parameter('mom_obs_gain').value} | "
            f"rate_max={self.get_parameter('tau_ext_max_rate').value} Nm/s"
        )

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
    # Main update loop
    # ─────────────────────────────────────────────────────────

    def _update(self):

        if not (self._js_received and self._ff_received and
                self._tau_m_received and self._traj_received and
                self._tau_ext_received):
            return

        dt = self._dt

        # ── PLANT PARAMETERS ─────────────────────────────────────────
        fric_visc = self.get_parameter('fric_visc').value
        fric_coul = self.get_parameter('fric_coul').value
        fric_eps = self.get_parameter('fric_eps').value
        damping = self.get_parameter('damping_theta').value
        alpha = self.get_parameter('residual_filter_alpha').value

        # ─────────────────────────────────────────────────────────────
        # OBSERVER 1 — Luenberger
        # ─────────────────────────────────────────────────────────────
        # Gain placement: given desired poles p1, p2, the observer gains
        # l1, l2 are derived from the characteristic polynomial of (A - L·C).

        p1 = self.get_parameter('obs1_pole_1').value
        p2 = self.get_parameter('obs1_pole_2').value

        D_lin = fric_visc + damping
        D_over_M = D_lin / self._M_eff

        l1 = -(p1 + p2) - D_over_M
        l2 = p1 * p2 - l1 * D_over_M

        if not self._obs1_initialized:
            self._theta_hat = self._theta_meas
            self._theta_dot_hat = self._theta_dot_meas
            self._obs1_initialized = True

        # Full friction on θ̇_hat (used in the Luenberger model)
        tau_fric_visc = fric_visc * self._theta_dot_hat
        tau_fric_coul = fric_coul * math.tanh(self._theta_dot_hat / fric_eps)
        tau_damp = damping * self._theta_dot_hat
        tau_dissipative = tau_fric_visc + tau_fric_coul + tau_damp

        theta_ddot_model = (
            self._tau_m
            + self._tau_pass
            + self._tau_ext_meas
            - self._proj
            - tau_dissipative
        ) / self._M_eff

        innov = self._theta_meas - self._theta_hat
        state_res = innov

        # Explicit Euler update (correct order: use old states to compute new ones)
        theta_dot_hat_new = (
            self._theta_dot_hat + dt * (theta_ddot_model + l2 * innov)
        )
        theta_hat_new = (
            self._theta_hat + dt * (self._theta_dot_hat + l1 * innov)
        )
        self._theta_dot_hat = theta_dot_hat_new
        self._theta_hat = theta_hat_new

        # ─────────────────────────────────────────────────────────────
        # OBSERVER 2 — Admittance Inversion
        # ─────────────────────────────────────────────────────────────
        # Reconstruct τ̂_ext by inverting the admittance model:
        #   τ̂_ext = M_v·θ̈_ref + D_v·θ̇_ref + K_v·(θ_ref - θ_eq)
        # Residual = τ_ext_meas - τ̂_ext
        # Invalid when the reference is saturated (admittance at a limit).

        Mv = self.get_parameter('adm_M_virt').value
        Dv = self.get_parameter('adm_D_virt').value
        Kv = self.get_parameter('adm_K_virt').value
        theta_eq = self.get_parameter('adm_theta_eq').value
        deadband = self.get_parameter('adm_force_deadband').value
        th_min = self.get_parameter('adm_theta_ref_min').value
        th_max = self.get_parameter('adm_theta_ref_max').value
        dv_max = self.get_parameter('adm_theta_dot_max').value

        sat_margin = 1e-3
        ref_pos_saturated = (
            self._theta_ref <= th_min + sat_margin or
            self._theta_ref >= th_max - sat_margin
        )
        ref_vel_saturated = (
            abs(self._theta_dot_ref) >= dv_max - sat_margin
        )
        in_deadband = abs(self._tau_ext_meas) < deadband

        self._obs2_valid = not (ref_pos_saturated or ref_vel_saturated or in_deadband)

        if self._obs2_valid:
            self._tau_ext_hat = (
                Mv * self._theta_ddot_ref +
                Dv * self._theta_dot_ref +
                Kv * (self._theta_ref - theta_eq)
            )
            torque_res = self._tau_ext_meas - self._tau_ext_hat
        else:
            self._tau_ext_hat = float('nan')
            torque_res = 0.0

        res_max = self.get_parameter('torque_res_max').value
        torque_res = max(-res_max, min(res_max, torque_res))

        # ─────────────────────────────────────────────────────────────
        # OBSERVER 3 — Generalized Momentum
        # ─────────────────────────────────────────────────────────────
        #
        # Principle: p = M_eff · θ̇  (generalized momentum)
        #
        #   dp/dt = Ṁ_eff·θ̇ + τ_m + τ_pass + τ_ext_real − proj − friction
        #
        # The observer integrates the known terms and compares with p:
        #
        #   σ̇ = τ_known + r_mom         σ(0) = p(0)
        #   r_mom = K_mom · (p − σ)
        #
        # At steady state:  r_mom → τ_ext_real − τ_ext_meas
        #
        # The Ṁ_eff·θ̇ term (inertia variation with configuration) is
        # compensated numerically:
        #   Δp_M ≈ (M_eff_k − M_eff_{k-1}) · θ̇

        K_mom = self.get_parameter('mom_obs_gain').value

        p_now = self._M_eff * self._theta_dot_meas

        if not self._mom_initialized:
            self._mom_sigma = p_now
            self._M_eff_prev = self._M_eff
            self._mom_initialized = True

        # Friction computed on MEASURED θ̇ (consistent with p = M·θ̇_meas)
        tau_fric_visc_m = fric_visc * self._theta_dot_meas
        tau_fric_coul_m = fric_coul * math.tanh(self._theta_dot_meas / fric_eps)
        tau_damp_m = damping * self._theta_dot_meas
        tau_dissipative_m = tau_fric_visc_m + tau_fric_coul_m + tau_damp_m

        # Known terms of dp/dt
        tau_known = (
            self._tau_m
            + self._tau_pass
            + self._tau_ext_meas
            - self._proj
            - tau_dissipative_m
        )

        # Inertia variation compensation: ∫ Ṁ·θ̇ dt ≈ ΔM · θ̇
        delta_p_M = (self._M_eff - self._M_eff_prev) * self._theta_dot_meas

        # Integrator update
        self._mom_sigma += (tau_known + self._r_momentum) * dt + delta_p_M

        # Momentum residual
        self._r_momentum = K_mom * (p_now - self._mom_sigma)

        self._M_eff_prev = self._M_eff

        # ─────────────────────────────────────────────────────────────
        # GUARD 4 — τ_ext rate-of-change
        # ─────────────────────────────────────────────────────────────

        max_rate = self.get_parameter('tau_ext_max_rate').value

        if not self._rate_guard_initialized:
            self._tau_ext_prev = self._tau_ext_meas
            self._rate_guard_initialized = True

        self._tau_ext_rate = abs(self._tau_ext_meas - self._tau_ext_prev) / dt
        self._rate_alarm = self._tau_ext_rate > max_rate
        self._tau_ext_prev = self._tau_ext_meas

        # ── RESIDUAL FILTERS ─────────────────────────────────────────
        self._state_res_filtered = ema(self._state_res_filtered, state_res, alpha)
        self._torque_res_filtered = ema(self._torque_res_filtered, torque_res, alpha)

        self._state_res_buf.append(state_res ** 2)
        self._torque_res_buf.append(torque_res ** 2)

        state_rms = rms_from_buf(self._state_res_buf)
        torque_rms = rms_from_buf(self._torque_res_buf)

        # ── PUBLISH ──────────────────────────────────────────────────

        msg = Float64()
        msg.data = state_res
        self._pub_state.publish(msg)

        msg = Float64()
        msg.data = torque_res
        self._pub_torque.publish(msg)

        msg = Float64()
        msg.data = self._r_momentum
        self._pub_momentum.publish(msg)

        msg = Float64()
        msg.data = 1.0 if self._rate_alarm else 0.0
        self._pub_rate_alarm.publish(msg)

        msg = Float64()
        msg.data = state_rms
        self._pub_state_rms.publish(msg)

        msg = Float64()
        msg.data = torque_rms
        self._pub_torque_rms.publish(msg)

        # Debug topic layout (21 fields):
        # [0]  theta_hat
        # [1]  theta_dot_hat
        # [2]  state_residual (innovation)
        # [3]  tau_ext_hat (0.0 if obs2 invalid)
        # [4]  torque_residual (obs2, clamped)
        # [5]  state_res_filtered (EMA)
        # [6]  torque_res_filtered (EMA)
        # [7]  state_rms
        # [8]  torque_rms
        # [9]  theta_meas
        # [10] theta_dot_meas
        # [11] tau_m
        # [12] tau_ext_meas
        # [13] theta_ref
        # [14] theta_dot_ref
        # [15] theta_ddot_ref
        # [16] obs2_valid (1.0 / 0.0)
        # [17] r_momentum (obs3)
        # [18] tau_ext_rate [Nm/s]
        # [19] rate_alarm (1.0 / 0.0)
        # [20] p_momentum (M_eff · θ̇)
        dbg = Float64MultiArray()
        dbg.data = [
            self._theta_hat,                                    # 0
            self._theta_dot_hat,                                # 1
            state_res,                                          # 2
            self._tau_ext_hat if self._obs2_valid else 0.0,     # 3
            torque_res,                                         # 4
            self._state_res_filtered,                           # 5
            self._torque_res_filtered,                          # 6
            state_rms,                                          # 7
            torque_rms,                                         # 8
            self._theta_meas,                                   # 9
            self._theta_dot_meas,                               # 10
            self._tau_m,                                        # 11
            self._tau_ext_meas,                                 # 12
            self._theta_ref,                                    # 13
            self._theta_dot_ref,                                # 14
            self._theta_ddot_ref,                               # 15
            1.0 if self._obs2_valid else 0.0,                   # 16
            self._r_momentum,                                   # 17
            self._tau_ext_rate,                                 # 18
            1.0 if self._rate_alarm else 0.0,                   # 19
            p_now,                                              # 20
        ]
        self._pub_debug.publish(dbg)


def main():
    rclpy.init()
    node = ObserverNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
