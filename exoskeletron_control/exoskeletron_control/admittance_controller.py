#!/usr/bin/env python3
"""
admittance_controller.py
========================
Outer-loop admittance controller per la dinamica ridotta dell'esoscheletro.

Principio di funzionamento
---------------------------
Il controllore di ammittanza implementa un modello virtuale massa-smorzatore-molla
che converte la forza dell'utente in un riferimento di traiettoria per l'inner loop.

La forza di input è tau_ext_theta [Nm], pubblicata dal nodo della dinamica su
/exo_dynamics/tau_ext_theta. Questo scalare rappresenta già la proiezione corretta
della wrench utente sul DOF attivo (theta = rev_crank):

    tau_ext_theta = B^T * J^T * W_user

dove B = dq/dtheta è il vettore cinematico istantaneo e J è il Jacobiano del frame
di contatto. La proiezione è quindi aggiornata ad ogni step dal plant, tenendo conto
della configurazione corrente del meccanismo.

Modello virtuale
----------------
    M * θ̈_v + D * θ̇_v + K * (θ_v - θ_eq) = tau_ext_theta

    θ̈_v = (tau_ext_theta - D * θ̇_v - K * (θ_v - θ_eq)) / M

Integrazione con Eulero esplicito:
    θ̇_v  ← θ̇_v + θ̈_v * dt
    θ_v  ← θ_v + θ̇_v * dt

L'output [θ_v, θ̇_v, θ̈_v] viene pubblicato su /trajectory_ref e consumato
direttamente dall'inner loop (trajectory_controller.py).

Schema a blocchi
----------------
  /exo_dynamics/tau_ext_theta  →  [Modello M-D-K]  →  /trajectory_ref
  /joint_states                →  (init θ_v)

Topics
------
  Sub:  /exo_dynamics/tau_ext_theta   std_msgs/Float64      forza utente proiettata su theta
        /joint_states                 sensor_msgs/JointState (per inizializzazione θ_v)
  Pub:  /trajectory_ref               std_msgs/Float64MultiArray  [θ_ref, θ̇_ref, θ̈_ref]
        /admittance/debug             std_msgs/Float64MultiArray  (diagnostica)

Parametri ROS2
--------------
  joint_name     (str,   default 'rev_crank')  joint da cui leggere theta iniziale
  M_virt         (float, default 0.5)   [Nm*s²/rad]  inerzia virtuale
  D_virt         (float, default 5.0)   [Nm*s/rad]   smorzamento virtuale
  K_virt         (float, default 2.0)   [Nm/rad]     rigidezza virtuale
  theta_eq       (float, default 0.0)   [rad]        posizione di equilibrio della molla
  force_deadband (float, default 0.1)   [Nm]         soglia sotto cui tau_ext_theta è zero
  theta_ref_min  (float, default -2.0)  [rad]        clamp inferiore su theta_ref
  theta_ref_max  (float, default  2.0)  [rad]        clamp superiore su theta_ref
  theta_dot_max  (float, default  5.0)  [rad/s]      saturazione velocità virtuale
  publish_rate   (float, default 200.0) [Hz]         frequenza del loop di controllo
  init_from_js   (bool,  default True)              inizializza θ_v da /joint_states

Note sul tuning
---------------
  - Aumentare D_virt per smorzare oscillazioni del riferimento
  - Aumentare K_virt per avere un comportamento più "rigido" (torna a theta_eq)
  - Ridurre M_virt per una risposta più reattiva alla forza utente
  - force_deadband filtra il rumore di stima su tau_ext_theta
"""

import rclpy
from rclpy.node import Node
import numpy as np

from std_msgs.msg import Float64, Float64MultiArray
from sensor_msgs.msg import JointState


class AdmittanceController(Node):

    def __init__(self):
        super().__init__('admittance_controller')

        # ============================================================
        # PARAMETRI
        # ============================================================
        self.declare_parameter('joint_name',     'rev_crank')
        self.declare_parameter('M_virt',          0.5)
        self.declare_parameter('D_virt',          5.0)
        self.declare_parameter('K_virt',          2.0)
        self.declare_parameter('theta_eq',        0.0)
        self.declare_parameter('force_deadband',  0.1)
        self.declare_parameter('theta_ref_min',  -2.0)
        self.declare_parameter('theta_ref_max',   2.0)
        self.declare_parameter('theta_dot_max',   5.0)
        self.declare_parameter('publish_rate',  200.0)
        self.declare_parameter('init_from_js',   True)

        self.joint_name   = str(self.get_parameter('joint_name').value)
        publish_rate      = float(self.get_parameter('publish_rate').value)
        self.init_from_js = bool(self.get_parameter('init_from_js').value)

        # ============================================================
        # STATO VIRTUALE
        # ============================================================
        self.theta_v      = 0.0   # posizione virtuale  → theta_ref
        self.theta_dot_v  = 0.0   # velocità virtuale   → theta_dot_ref
        self.theta_ddot_v = 0.0   # accelerazione virtuale → theta_ddot_ref

        # Flag inizializzazione: se init_from_js=True aspetta il primo /joint_states
        self._initialized = not self.init_from_js

        # Ingresso: forza utente proiettata su theta (scalare, già in [Nm])
        self.tau_ext_theta = 0.0
        self.force_received = False

        # dt del loop di controllo
        self._dt = 1.0 / publish_rate

        # ============================================================
        # ROS I/O
        # ============================================================

        # Forza utente proiettata su theta: input diretto al modello M-D-K
        self.sub_force = self.create_subscription(
            Float64,
            '/exo_dynamics/tau_ext_theta',
            self._force_cb,
            10
        )

        # Joint states: solo per inizializzare theta_v alla posizione attuale
        self.sub_js = self.create_subscription(
            JointState,
            '/joint_states',
            self._js_cb,
            10
        )

        self.pub_ref  = self.create_publisher(Float64MultiArray, '/trajectory_ref', 10)
        self.pub_diag = self.create_publisher(Float64MultiArray, '/admittance/debug', 10)

        self.timer = self.create_timer(self._dt, self._control_loop)

        self.get_logger().info(
            f"AdmittanceController avviato | "
            f"M={self.get_parameter('M_virt').value} "
            f"D={self.get_parameter('D_virt').value} "
            f"K={self.get_parameter('K_virt').value} "
            f"theta_eq={self.get_parameter('theta_eq').value} "
            f"init_from_js={self.init_from_js}"
        )

    # ============================================================
    # CALLBACKS
    # ============================================================

    def _js_cb(self, msg: JointState):
        """
        Inizializza theta_v dalla posizione attuale del crank (una sola volta).
        Evita un transiente iniziale brusco quando il controllore parte
        con il meccanismo già in una configurazione non nulla.
        """
        if self._initialized:
            return
        try:
            idx = msg.name.index(self.joint_name)
        except ValueError:
            return
        if idx < len(msg.position):
            self.theta_v      = float(msg.position[idx])
            self.theta_dot_v  = 0.0
            self._initialized = True
            self.get_logger().info(
                f"theta_v inizializzato da /joint_states: {self.theta_v:.4f} rad"
            )

    def _force_cb(self, msg: Float64):
        """
        Riceve tau_ext_theta [Nm] dal nodo della dinamica.
        È già la proiezione corretta della forza utente su theta:
            tau_ext_theta = B^T * J^T * W_user
        Non è necessaria nessuna ulteriore proiezione o conversione.
        """
        self.tau_ext_theta = float(msg.data)
        self.force_received = True

    # ============================================================
    # LOOP DI CONTROLLO
    # ============================================================

    def _control_loop(self):
        """
        Integra il modello virtuale M-D-K e pubblica il riferimento di traiettoria.

        Equazione del moto virtuale:
            M * θ̈_v = tau_ext_theta - D * θ̇_v - K * (θ_v - θ_eq)

        Caso degenere M ≈ 0 (smorzatore-molla puro):
            θ̇_v = (tau_ext_theta - K * (θ_v - θ_eq)) / D
        """
        if not self._initialized:
            return

        # Lettura parametri (aggiornabili a runtime via ros2 param set)
        M        = float(self.get_parameter('M_virt').value)
        D        = float(self.get_parameter('D_virt').value)
        K        = float(self.get_parameter('K_virt').value)
        theta_eq = float(self.get_parameter('theta_eq').value)
        db       = float(self.get_parameter('force_deadband').value)
        th_min   = float(self.get_parameter('theta_ref_min').value)
        th_max   = float(self.get_parameter('theta_ref_max').value)
        dv_max   = float(self.get_parameter('theta_dot_max').value)

        # Deadband: filtra il rumore di stima su tau_ext_theta
        tau_in = self.tau_ext_theta
        if abs(tau_in) < db:
            tau_in = 0.0

        # Termine elastico: richiamo verso theta_eq
        tau_spring = K * (self.theta_v - theta_eq)

        # Termine viscoso: smorzamento della velocità virtuale
        tau_damper = D * self.theta_dot_v

        if M < 1e-6:
            # Caso degenere: nessuna inerzia virtuale
            # theta_dot_v segue direttamente la forza netta (smorzatore + molla)
            if D < 1e-6:
                self.theta_dot_v  = 0.0
                self.theta_ddot_v = 0.0
            else:
                self.theta_dot_v  = (tau_in - tau_spring) / D
                self.theta_ddot_v = 0.0
        else:
            # Caso generale: M * θ̈_v = tau_in - tau_damper - tau_spring
            self.theta_ddot_v = (tau_in - tau_damper - tau_spring) / M

        # Integrazione Eulero esplicito
        self.theta_dot_v += self.theta_ddot_v * self._dt
        self.theta_dot_v  = float(np.clip(self.theta_dot_v, -dv_max, dv_max))
        self.theta_v     += self.theta_dot_v * self._dt
        self.theta_v      = float(np.clip(self.theta_v, th_min, th_max))

        # --------------------------------------------------------
        # Pubblica /trajectory_ref → [θ_ref, θ̇_ref, θ̈_ref]
        # Consumato da trajectory_controller.py (inner loop PD+)
        # --------------------------------------------------------
        ref = Float64MultiArray()
        ref.data = [
            float(self.theta_v),
            float(self.theta_dot_v),
            float(self.theta_ddot_v),
        ]
        self.pub_ref.publish(ref)

        # --------------------------------------------------------
        # Diagnostica /admittance/debug
        # Layout:
        #   [0]  theta_v        posizione virtuale (= theta_ref)
        #   [1]  theta_dot_v    velocità virtuale
        #   [2]  theta_ddot_v   accelerazione virtuale
        #   [3]  tau_ext_theta  forza utente (raw, prima della deadband)
        #   [4]  tau_in         forza utente (dopo deadband)
        #   [5]  tau_spring     termine elastico K*(theta_v - theta_eq)
        #   [6]  tau_damper     termine viscoso D*theta_dot_v
        #   [7]  theta_eq       punto di equilibrio corrente
        #   [8]  M              inerzia virtuale corrente
        #   [9]  D              smorzamento virtuale corrente
        #   [10] K              rigidezza virtuale corrente
        # --------------------------------------------------------
        diag = Float64MultiArray()
        diag.data = [
            float(self.theta_v),
            float(self.theta_dot_v),
            float(self.theta_ddot_v),
            float(self.tau_ext_theta),
            float(tau_in),
            float(tau_spring),
            float(tau_damper),
            float(theta_eq),
            float(M),
            float(D),
            float(K),
        ]
        self.pub_diag.publish(diag)


def main(args=None):
    rclpy.init(args=args)
    node = AdmittanceController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()