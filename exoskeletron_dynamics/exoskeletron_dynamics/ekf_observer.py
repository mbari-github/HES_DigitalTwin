#!/usr/bin/env python3
"""
ekf_observer.py
===============

Observer EKF con degradazione graceful per canali di osservazione.

Quando un canale sensore supera obs_timeout_sec senza aggiornamenti,
il filtro passa automaticamente in modalità predict-only per quello stato,
usando solo il modello dinamico. La covarianza P cresce nel tempo
indicando l'incertezza crescente sulla stima.

Stato stimato:
    x = [theta, theta_dot, tau_ext_theta]

Modello di processo:
    theta_ddot = (tau_m + tau_ext_theta + tau_pass - proj) / M_eff
    x_dot = [theta_dot,
             theta_ddot,
             0]   <- tau_ext_theta: random walk

Osservazioni (con degradazione individuale):
    z[0] = theta_meas       da /joint_states
    z[1] = theta_dot_meas   da /joint_states
    z[2] = tau_ext_meas     da /exo_dynamics/tau_ext_theta

Topics
------
Sub:
    /joint_states                   sensor_msgs/JointState
    /torque                         std_msgs/Float64
    /exo_dynamics/ff_terms          std_msgs/Float64MultiArray
    /exo_dynamics/tau_ext_theta     std_msgs/Float64
    /exo_dynamics/debug             std_msgs/Float64MultiArray (ground truth)

Pub:
    /observer/state_estimate        std_msgs/Float64MultiArray
        [theta_est, theta_dot_est, tau_ext_est]

    /observer/residual              std_msgs/Float64MultiArray
        [res_theta, res_theta_dot, res_tau_ext,          vs sensore
         res_theta_gt, res_theta_dot_gt, res_tau_ext_gt] vs ground truth

    /observer/covariance            std_msgs/Float64MultiArray
        [P[0,0], P[1,1], P[2,2]]

    /observer/sensor_health         std_msgs/Float64MultiArray
        [theta_ok, theta_dot_ok, tau_ext_ok,
         theta_age, theta_dot_age, tau_ext_age,
         timeout_sec]

    /observer/debug                 std_msgs/Float64MultiArray

Parametri ROS2
--------------
    joint_name              (str,   default 'rev_crank')
    dt                      (float, default 0.005)
    publish_rate            (float, default 200.0)

    # Timeout canali osservazione [s]
    obs_timeout_sec         (float, default 0.5)

    # Covarianza iniziale
    P0_theta                (float, default 1e-4)
    P0_theta_dot            (float, default 1e-3)
    P0_tau_ext              (float, default 1.0)

    # Rumore di processo Q
    q_theta                 (float, default 1e-6)
    q_theta_dot             (float, default 1e-4)
    q_tau_ext               (float, default 0.1)

    # Rumore di misura R
    r_theta                 (float, default 1e-4)
    r_theta_dot             (float, default 1e-3)
    r_tau_ext               (float, default 0.01)

    # Sicurezza numerica
    M_eff_min               (float, default 1e-4)

    # Soglie covarianza per affidabilità stima
    # Se P[i,i] supera la soglia, la stima di quello stato
    # non è più considerata affidabile
    P_theta_unreliable      (float, default 0.01)
    P_theta_dot_unreliable  (float, default 0.1)
    P_tau_ext_unreliable    (float, default 10.0)
"""

import numpy as np
import rclpy
from rclpy.node import Node

from std_msgs.msg import Float64, Float64MultiArray
from sensor_msgs.msg import JointState


# Indici canali osservazione
OBS_THETA     = 0
OBS_THETA_DOT = 1
OBS_TAU_EXT   = 2
OBS_NAMES     = ['theta', 'theta_dot', 'tau_ext']


class EKFObserver(Node):

    def __init__(self):
        super().__init__('ekf_observer')

        # ============================================================
        # PARAMETRI
        # ============================================================
        self.declare_parameter('joint_name',           'rev_crank')
        self.declare_parameter('dt',                   0.005)
        self.declare_parameter('publish_rate',         200.0)
        self.declare_parameter('obs_timeout_sec',      0.5)

        self.declare_parameter('P0_theta',             1e-4)
        self.declare_parameter('P0_theta_dot',         1e-3)
        self.declare_parameter('P0_tau_ext',           1.0)

        self.declare_parameter('q_theta',              1e-6)
        self.declare_parameter('q_theta_dot',          1e-4)
        self.declare_parameter('q_tau_ext',            0.1)

        self.declare_parameter('r_theta',              1e-4)
        self.declare_parameter('r_theta_dot',          1e-3)
        self.declare_parameter('r_tau_ext',            0.01)

        self.declare_parameter('M_eff_min',            1e-4)

        self.declare_parameter('P_theta_unreliable',     0.01)
        self.declare_parameter('P_theta_dot_unreliable', 0.1)
        self.declare_parameter('P_tau_ext_unreliable',   10.0)

        self.joint_name      = str(self.get_parameter('joint_name').value)
        self.dt              = float(self.get_parameter('dt').value)
        publish_rate         = float(self.get_parameter('publish_rate').value)
        self.obs_timeout     = float(self.get_parameter('obs_timeout_sec').value)
        self.M_eff_min       = float(self.get_parameter('M_eff_min').value)

        self.P_unreliable = np.array([
            float(self.get_parameter('P_theta_unreliable').value),
            float(self.get_parameter('P_theta_dot_unreliable').value),
            float(self.get_parameter('P_tau_ext_unreliable').value),
        ])

        # ============================================================
        # STATO EKF
        # ============================================================
        self.x = np.zeros(3)

        self.P = np.diag([
            float(self.get_parameter('P0_theta').value),
            float(self.get_parameter('P0_theta_dot').value),
            float(self.get_parameter('P0_tau_ext').value),
        ])

        self.Q = np.diag([
            float(self.get_parameter('q_theta').value),
            float(self.get_parameter('q_theta_dot').value),
            float(self.get_parameter('q_tau_ext').value),
        ])

        self.R_full = np.diag([
            float(self.get_parameter('r_theta').value),
            float(self.get_parameter('r_theta_dot').value),
            float(self.get_parameter('r_tau_ext').value),
        ])

        # ============================================================
        # TIMESTAMP CANALI OSSERVAZIONE
        # None = mai ricevuto
        # ============================================================
        self._last_rx: list = [None, None, None]

        # Valori osservati (congelati all'ultimo valido)
        self.z = np.zeros(3)

        # ============================================================
        # INGRESSI NOTI
        # ============================================================
        self.tau_m:    float = 0.0
        self.M_eff:    float = 1.0
        self.proj:     float = 0.0
        self.tau_pass: float = 0.0
        self.ff_received: bool = False

        # Ground truth
        self.gt = np.zeros(3)
        self.gt_received: bool = False

        self._initialized: bool = False

        # ============================================================
        # ROS I/O
        # ============================================================
        self.create_subscription(
            JointState, '/joint_states', self._cb_js, 10)
        self.create_subscription(
            Float64, '/torque', self._cb_tau, 10)
        self.create_subscription(
            Float64MultiArray, '/exo_dynamics/ff_terms', self._cb_ff, 10)
        self.create_subscription(
            Float64, '/exo_dynamics/tau_ext_theta', self._cb_tau_ext, 10)
        self.create_subscription(
            Float64MultiArray, '/exo_dynamics/debug', self._cb_debug_gt, 10)

        self.pub_estimate = self.create_publisher(
            Float64MultiArray, '/observer/state_estimate', 10)
        self.pub_residual = self.create_publisher(
            Float64MultiArray, '/observer/residual', 10)
        self.pub_cov = self.create_publisher(
            Float64MultiArray, '/observer/covariance', 10)
        self.pub_health = self.create_publisher(
            Float64MultiArray, '/observer/sensor_health', 10)
        self.pub_debug = self.create_publisher(
            Float64MultiArray, '/observer/debug', 10)

        self.timer = self.create_timer(1.0 / publish_rate, self._loop)

        self.get_logger().info(
            f'EKFObserver avviato | joint={self.joint_name} | '
            f'dt={self.dt}s | obs_timeout={self.obs_timeout}s\n'
            f'  Q  = diag{np.diag(self.Q).tolist()}\n'
            f'  R  = diag{np.diag(self.R_full).tolist()}\n'
            f'  P0 = diag{np.diag(self.P).tolist()}\n'
            f'  P_unreliable = {self.P_unreliable.tolist()}'
        )

    # ================================================================
    # CALLBACKS
    # ================================================================

    def _touch(self, idx: int):
        self._last_rx[idx] = self.get_clock().now().nanoseconds * 1e-9

    def _cb_js(self, msg: JointState):
        try:
            i = msg.name.index(self.joint_name)
        except ValueError:
            return
        if i < len(msg.position):
            self.z[OBS_THETA] = float(msg.position[i])
            self._touch(OBS_THETA)
        if i < len(msg.velocity):
            self.z[OBS_THETA_DOT] = float(msg.velocity[i])
            self._touch(OBS_THETA_DOT)

    def _cb_tau(self, msg: Float64):
        self.tau_m = float(msg.data)

    def _cb_ff(self, msg: Float64MultiArray):
        d = msg.data
        if len(d) > 0:
            self.M_eff = max(float(d[0]), self.M_eff_min)
        if len(d) > 1:
            self.proj = float(d[1])
        if len(d) > 3:
            self.tau_pass = float(d[3])
        self.ff_received = True

    def _cb_tau_ext(self, msg: Float64):
        self.z[OBS_TAU_EXT] = float(msg.data)
        self._touch(OBS_TAU_EXT)

    def _cb_debug_gt(self, msg: Float64MultiArray):
        d = msg.data
        if len(d) > 0:  self.gt[0] = float(d[0])
        if len(d) > 1:  self.gt[1] = float(d[1])
        if len(d) > 12: self.gt[2] = float(d[12])
        self.gt_received = True

    # ================================================================
    # CHANNEL HEALTH
    # ================================================================

    def _channel_status(self):
        """
        Restituisce per ogni canale:
          - alive: bool (True se entro timeout)
          - age:   float (secondi dall'ultimo messaggio, -1 se mai ricevuto)
        """
        now = self.get_clock().now().nanoseconds * 1e-9
        status = []
        for i in range(3):
            t = self._last_rx[i]
            if t is None:
                status.append((False, -1.0))
            else:
                age = now - t
                status.append((age <= self.obs_timeout, age))
        return status

    # ================================================================
    # EKF — PREDICT
    # ================================================================

    def _predict(self):
        theta     = self.x[0]
        theta_dot = self.x[1]
        tau_ext   = self.x[2]

        theta_ddot = (
            self.tau_m + tau_ext + self.tau_pass - self.proj
        ) / self.M_eff

        x_pred = np.array([
            theta     + theta_dot  * self.dt,
            theta_dot + theta_ddot * self.dt,
            tau_ext,
        ])

        F = np.array([
            [1.0, self.dt,  0.0                  ],
            [0.0, 1.0,      self.dt / self.M_eff  ],
            [0.0, 0.0,      1.0                  ],
        ])

        P_pred = F @ self.P @ F.T + self.Q
        return x_pred, P_pred

    # ================================================================
    # EKF — UPDATE PARZIALE
    # ================================================================

    def _update_partial(self, x_pred: np.ndarray, P_pred: np.ndarray,
                        alive: list):
        """
        Update che usa solo i canali vivi.

        Per ogni canale morto, la riga corrispondente di H viene rimossa
        e la stima evolve solo tramite predict per quello stato.
        Questo fa crescere P[i,i] per gli stati non osservati,
        comunicando l'incertezza crescente al sistema.

        Se nessun canale è vivo, restituisce la predizione pura.
        """
        active = [i for i, (ok, _) in enumerate(alive) if ok]

        if not active:
            # Predict-only: nessun sensore disponibile
            return x_pred, P_pred, np.full(3, float('nan'))

        # Costruisce H, z, R ridotti ai soli canali attivi
        H_red = np.eye(3)[active, :]          # shape (n_active, 3)
        z_red = self.z[active]                # shape (n_active,)
        R_red = self.R_full[np.ix_(active, active)]  # shape (n_active, n_active)

        innov_red = z_red - H_red @ x_pred

        S = H_red @ P_pred @ H_red.T + R_red
        K = P_pred @ H_red.T @ np.linalg.inv(S)

        x_upd = x_pred + K @ innov_red

        # Forma Joseph per stabilità numerica
        I_KH  = np.eye(3) - K @ H_red
        P_upd = I_KH @ P_pred @ I_KH.T + K @ R_red @ K.T

        # Ricostruisce innovazione full (nan per canali morti)
        innov_full = np.full(3, float('nan'))
        for j, i in enumerate(active):
            innov_full[i] = innov_red[j]

        return x_upd, P_upd, innov_full

    # ================================================================
    # LOOP PRINCIPALE
    # ================================================================

    def _loop(self):
        if not self.ff_received:
            return

        status = self._channel_status()
        alive  = [ok for ok, _ in status]

        # Aspetta almeno un canale vivo per inizializzare
        if not self._initialized:
            if not any(alive):
                return
            # Inizializza lo stato con le misure disponibili
            # Per i canali non ancora vivi usa zero
            self.x = self.z.copy()
            self._initialized = True
            self.get_logger().info(
                f'EKFObserver: inizializzato | '
                f'theta={self.x[0]:.4f} '
                f'theta_dot={self.x[1]:.4f} '
                f'tau_ext={self.x[2]:.4f}'
            )
            return

        # ── Log canali morti (solo al cambio di stato) ───────────────
        for i, (ok, age) in enumerate(status):
            if not ok and age > 0:
                self.get_logger().warn(
                    f'EKFObserver: canale [{OBS_NAMES[i]}] stale '
                    f'(age={age:.3f}s) → predict-only per questo stato',
                    throttle_duration_sec=2.0
                )

        # ── Predict ─────────────────────────────────────────────────
        x_pred, P_pred = self._predict()

        # ── Update parziale (solo canali vivi) ───────────────────────
        x_upd, P_upd, innov = self._update_partial(
            x_pred, P_pred, status)

        self.x = x_upd
        self.P = P_upd

        # ── Affidabilità stima ───────────────────────────────────────
        reliable = [
            float(self.P[i, i] < self.P_unreliable[i])
            for i in range(3)
        ]

        # ── Pubblica stima ───────────────────────────────────────────
        est = Float64MultiArray()
        est.data = [float(self.x[0]),
                    float(self.x[1]),
                    float(self.x[2])]
        self.pub_estimate.publish(est)

        # ── Pubblica covarianza ──────────────────────────────────────
        cov = Float64MultiArray()
        cov.data = [float(self.P[0, 0]),
                    float(self.P[1, 1]),
                    float(self.P[2, 2])]
        self.pub_cov.publish(cov)

        # ── Pubblica sensor health ───────────────────────────────────
        # Layout:
        # [0]  theta_ok          1.0=vivo 0.0=stale
        # [1]  theta_dot_ok
        # [2]  tau_ext_ok
        # [3]  theta_age_sec     età del messaggio (-1 se mai ricevuto)
        # [4]  theta_dot_age_sec
        # [5]  tau_ext_age_sec
        # [6]  timeout_sec
        # [7]  theta_reliable    1.0 se P[i,i] < soglia
        # [8]  theta_dot_reliable
        # [9]  tau_ext_reliable
        health = Float64MultiArray()
        health.data = [
            1.0 if alive[0] else 0.0,
            1.0 if alive[1] else 0.0,
            1.0 if alive[2] else 0.0,
            float(status[0][1]),
            float(status[1][1]),
            float(status[2][1]),
            float(self.obs_timeout),
            reliable[0],
            reliable[1],
            reliable[2],
        ]
        self.pub_health.publish(health)

        # ── Pubblica residui ─────────────────────────────────────────
        # vs sensore (solo canali vivi, nan per canali morti)
        res_vs_meas = np.full(3, float('nan'))
        for i in range(3):
            if alive[i]:
                res_vs_meas[i] = self.z[i] - self.x[i]

        # vs ground truth
        if self.gt_received:
            res_vs_gt = self.gt - self.x
        else:
            res_vs_gt = np.full(3, float('nan'))

        res = Float64MultiArray()
        res.data = [
            float(res_vs_meas[0]),
            float(res_vs_meas[1]),
            float(res_vs_meas[2]),
            float(res_vs_gt[0]),
            float(res_vs_gt[1]),
            float(res_vs_gt[2]),
        ]
        self.pub_residual.publish(res)

        # ── Pubblica debug esteso ────────────────────────────────────
        # [0-2]   stato stimato
        # [3-5]   osservazioni correnti
        # [6-8]   innovazione (nan se canale morto)
        # [9-11]  diagonale P
        # [12-15] termini dinamici
        # [16]    theta_ddot stimata
        # [17-19] residui vs ground truth
        # [20-22] affidabilità per stato

        theta_ddot_est = (
            self.tau_m + self.x[2] + self.tau_pass - self.proj
        ) / self.M_eff

        dbg = Float64MultiArray()
        dbg.data = [
            float(self.x[0]),
            float(self.x[1]),
            float(self.x[2]),
            float(self.z[0]),
            float(self.z[1]),
            float(self.z[2]),
            float(innov[0]),
            float(innov[1]),
            float(innov[2]),
            float(self.P[0, 0]),
            float(self.P[1, 1]),
            float(self.P[2, 2]),
            float(self.M_eff),
            float(self.proj),
            float(self.tau_pass),
            float(self.tau_m),
            float(theta_ddot_est),
            float(res_vs_gt[0]),
            float(res_vs_gt[1]),
            float(res_vs_gt[2]),
            reliable[0],
            reliable[1],
            reliable[2],
        ]
        self.pub_debug.publish(dbg)


def main(args=None):
    rclpy.init(args=args)
    node = EKFObserver()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()