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
  /joint_states                →  (init θ_v e resume da freeze)
  /admittance/freeze           →  (congela/sblocca l'integrazione)

Topics
------
  Sub:  /exo_dynamics/tau_ext_theta   std_msgs/Float64
        /joint_states                 sensor_msgs/JointState
        /admittance/freeze            std_msgs/Bool
  Pub:  /trajectory_ref               std_msgs/Float64MultiArray  [θ_ref, θ̇_ref, θ̈_ref]
        /admittance/debug             std_msgs/Float64MultiArray

Parametri ROS2
--------------
  joint_name     (str,   default 'rev_crank')
  M_virt         (float, default 0.5)
  D_virt         (float, default 5.0)
  K_virt         (float, default 2.0)
  theta_eq       (float, default 0.0)
  force_deadband (float, default 0.1)
  theta_ref_min  (float, default -2.0)
  theta_ref_max  (float, default  2.0)
  theta_dot_max  (float, default  5.0)
  publish_rate   (float, default 200.0)
  init_from_js   (bool,  default True)
"""

import numpy as np
import rclpy
from rclpy.node import Node

from std_msgs.msg import Float64, Float64MultiArray, Bool
from sensor_msgs.msg import JointState


class AdmittanceController(Node):

    def __init__(self):
        super().__init__('admittance_controller')

        self.declare_parameter('joint_name',     'rev_crank')
        self.declare_parameter('M_virt',          0.5)
        self.declare_parameter('D_virt',          5.0)
        self.declare_parameter('K_virt',          2.0)
        self.declare_parameter('theta_eq',        0.0)
        self.declare_parameter('force_deadband',  0.0)
        self.declare_parameter('theta_ref_min',  -2.0)
        self.declare_parameter('theta_ref_max',   2.0)
        self.declare_parameter('theta_dot_max',   5.0)
        self.declare_parameter('publish_rate',  200.0)
        self.declare_parameter('init_from_js',   True)

        publish_rate    = float(self.get_parameter('publish_rate').value)
        self.joint_name = str(self.get_parameter('joint_name').value)
        self.init_from_js = bool(self.get_parameter('init_from_js').value)

        # Stato interno modello virtuale
        self.theta_v      = 0.0
        self.theta_dot_v  = 0.0
        self.theta_ddot_v = 0.0

        # Inizializzazione da joint_states
        self._initialized = not self.init_from_js

        # FIX Bug #3/#4: stato di freeze
        # True  → l'integrazione è sospesa, theta_v e theta_dot_v sono congelati
        # False → funzionamento normale
        self._frozen = False

        # Ultima posizione reale nota (usata per resettare theta_v al resume)
        self._last_real_theta: float | None = None

        self.tau_ext_theta  = 0.0
        self.force_received = False

        self._dt = 1.0 / publish_rate

        # ── Subscribers ──────────────────────────────────────────────
        self.sub_force = self.create_subscription(
            Float64,
            '/exo_dynamics/tau_ext_theta',
            self._force_cb,
            10
        )
        self.sub_js = self.create_subscription(
            JointState,
            '/joint_states',
            self._js_cb,
            10
        )
        # FIX Bug #4: sottoscrizione al topic di freeze pubblicato dal bridge
        self.sub_freeze = self.create_subscription(
            Bool,
            '/admittance/freeze',
            self._freeze_cb,
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
        Aggiorna anche _last_real_theta ad ogni tick (usato al resume da freeze).
        """
        try:
            idx = msg.name.index(self.joint_name)
        except ValueError:
            return

        if idx < len(msg.position):
            self._last_real_theta = float(msg.position[idx])

            if not self._initialized:
                self.theta_v     = self._last_real_theta
                self.theta_dot_v = 0.0
                self._initialized = True
                self.get_logger().info(
                    f"theta_v inizializzato da /joint_states: {self.theta_v:.4f} rad"
                )

    def _force_cb(self, msg: Float64):
        self.tau_ext_theta = float(msg.data)
        self.force_received = True

    def _freeze_cb(self, msg: Bool):
        """
        FIX Bug #3/#4: riceve il comando di freeze/unfreeze dal bridge.

        freeze=True  → congela l'integrazione (bridge in STOP).
                        theta_v e theta_dot_v rimangono al valore corrente.
        freeze=False → sblocca l'integrazione (bridge esce da STOP).
                        IMPORTANTE: al resume, theta_v viene resettato alla
                        posizione reale corrente per evitare spike di coppia
                        dovuti al drift accumulato durante il freeze.
        """
        new_frozen = bool(msg.data)

        if new_frozen == self._frozen:
            return  # nessun cambiamento di stato

        if new_frozen:
            self._frozen = True
            self.get_logger().warn(
                f'AdmittanceController: FREEZE | theta_v={self.theta_v:.4f} rad'
            )
        else:
            # Resume: riallinea theta_v alla posizione reale per evitare spike
            if self._last_real_theta is not None:
                old_theta_v = self.theta_v
                self.theta_v     = self._last_real_theta
                self.theta_dot_v = 0.0
                self.get_logger().info(
                    f'AdmittanceController: UNFREEZE | '
                    f'theta_v riallineato: {old_theta_v:.4f} → {self.theta_v:.4f} rad'
                )
            else:
                self.get_logger().warn(
                    'AdmittanceController: UNFREEZE | _last_real_theta non disponibile, '
                    'theta_v mantenuto invariato'
                )
            self._frozen = False

    # ============================================================
    # LOOP DI CONTROLLO
    # ============================================================

    def _control_loop(self):
        if not self._initialized:
            return

        # FIX Bug #3: se il bridge è in STOP, non integrare.
        # Pubblica comunque il riferimento congelato per mantenere
        # attivo il topic (evita watchdog del trajectory controller).
        if self._frozen:
            ref = Float64MultiArray()
            ref.data = [float(self.theta_v), 0.0, 0.0]
            self.pub_ref.publish(ref)
            return

        M        = float(self.get_parameter('M_virt').value)
        D        = float(self.get_parameter('D_virt').value)
        K        = float(self.get_parameter('K_virt').value)
        theta_eq = float(self.get_parameter('theta_eq').value)
        db       = float(self.get_parameter('force_deadband').value)
        th_min   = float(self.get_parameter('theta_ref_min').value)
        th_max   = float(self.get_parameter('theta_ref_max').value)
        dv_max   = float(self.get_parameter('theta_dot_max').value)

        tau_in = self.tau_ext_theta
        if abs(tau_in) < db:
            tau_in = 0.0

        tau_spring = K * (self.theta_v - theta_eq)
        tau_damper = D * self.theta_dot_v

        if M < 1e-6:
            if D < 1e-6:
                self.theta_dot_v  = 0.0
                self.theta_ddot_v = 0.0
            else:
                self.theta_dot_v  = (tau_in - tau_spring) / D
                self.theta_ddot_v = 0.0
        else:
            self.theta_ddot_v = (tau_in - tau_damper - tau_spring) / M

        self.theta_dot_v += self.theta_ddot_v * self._dt
        self.theta_dot_v  = float(np.clip(self.theta_dot_v, -dv_max, dv_max))
        self.theta_v     += self.theta_dot_v * self._dt
        self.theta_v      = float(np.clip(self.theta_v, th_min, th_max))

        ref = Float64MultiArray()
        ref.data = [
            float(self.theta_v),
            float(self.theta_dot_v),
            float(self.theta_ddot_v),
        ]
        self.pub_ref.publish(ref)

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