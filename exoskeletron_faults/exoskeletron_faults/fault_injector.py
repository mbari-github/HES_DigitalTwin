#!/usr/bin/env python3
"""

Nodo ROS2 per la fault injection controllata sull'architettura del digital twin
dell'esoscheletro (plant + admittance controller + trajectory controller).

Principio di funzionamento
---------------------------
Il nodo si inserisce come proxy su uno dei quattro topic critici dell'architettura:

  CANALE 0 — /exo_dynamics/tau_ext_theta  (Float64)
    Simula un guasto nella stima della forza utente proiettata su theta.
    Es: sensore forza al polso degradato, bias nella stima della wrench.

  CANALE 1 — /trajectory_ref  (Float64MultiArray: [theta_ref, theta_dot_ref, theta_ddot_ref])
    Simula un guasto nell'outer loop (admittance controller bloccato,
    riferimento corrotto, oscillazione nel riferimento di posizione/velocità).

  CANALE 2 — /torque  (Float64)
    Simula un guasto nell'inner loop (trajectory controller).
    Corrisponde allo scenario "errore nel controllo di velocità/posizione di un dito"
    descritto dall'azienda: la coppia attuata non corrisponde a quella comandata.

  CANALE 3 — /joint_states  (sensor_msgs/JointState)
    Simula un guasto sul sensore di posizione (encoder) nel ramo di retroazione.
    Il fault viene applicato a theta (posizione) e/o theta_dot (velocità) del
    joint 'rev_crank', lasciando intatti tutti gli altri joint del messaggio.
    Es: drift dell'encoder, rumore sul segnale di posizione, lettura congelata.

Modalità di fault disponibili
------------------------------
  none    — trasparente, il nodo non altera il segnale
  offset  — somma un bias costante al segnale (o al primo elemento per MultiArray)
  noise   — aggiunge rumore gaussiano a campionamento continuo
  freeze  — blocca completamente la pubblicazione del topic faultato
  scale   — moltiplica il segnale per un fattore (es. 0.0 → coppia nulla)
  spike   — inietta un impulso singolo di ampiezza fault_magnitude, poi torna trasparente

La fault è attivabile/disattivabile a runtime senza riavviare il nodo:
    ros2 param set /fault_injector fault_active true
    ros2 param set /fault_injector fault_type offset
    ros2 param set /fault_injector fault_magnitude 2.0

Schema di remapping
--------------------
Il nodo si abbona al topic originale e pubblica su un topic "_faulted".
Per inserirlo nell'architettura senza modificare gli altri nodi, usare il
remapping ROS2 nel launch file:

    # Nel launch file:
    # 1. fault_injector si abbona a /torque (originale dal trajectory_controller)
    # 2. fault_injector pubblica su /torque_faulted
    # 3. il plant viene riavviato con remapping /torque → /torque_faulted

Topics
------
  CANALE 0:
    Sub:  /exo_dynamics/tau_ext_theta         std_msgs/Float64
    Pub:  /exo_dynamics/tau_ext_theta_faulted std_msgs/Float64

  CANALE 1:
    Sub:  /trajectory_ref                     std_msgs/Float64MultiArray
    Pub:  /trajectory_ref_faulted             std_msgs/Float64MultiArray

  CANALE 2:
    Sub:  /torque                             std_msgs/Float64
    Pub:  /torque_faulted                     std_msgs/Float64

  CANALE 3:
    Sub:  /joint_states                       sensor_msgs/JointState
    Pub:  /joint_states_faulted               sensor_msgs/JointState

  Sempre:
    Pub:  /fault_injector/status              std_msgs/Float64MultiArray
          Layout: [channel, fault_type_id, fault_active, fault_magnitude,
                   n_injections, last_raw, last_faulted, delta]

Parametri ROS2
--------------
  channel           (int,   default 2)        canale su cui iniettare il fault
                                              0=tau_ext_theta, 1=trajectory_ref,
                                              2=torque, 3=joint_states
  fault_active      (bool,  default False)    attiva/disattiva la fault injection
  fault_type        (str,   default 'offset') tipo di fault: none/offset/noise/freeze/scale/spike
  fault_magnitude   (float, default 1.0)      ampiezza del fault
  noise_std         (float, default 0.1)      deviazione standard del rumore (solo fault_type=noise)
  target_index      (int,   default 0)        indice del Float64MultiArray da corrompere (solo canale 1)
                                              0=theta_ref, 1=theta_dot_ref, 2=theta_ddot_ref
  fault_js_field    (str,   default 'position') campo JointState da corrompere (solo canale 3)
                                              'position' → theta, 'velocity' → theta_dot, 'both'
  joint_name        (str,   default 'rev_crank') nome del joint da corrompere (solo canale 3)
  publish_rate      (float, default 200.0)    Hz del topic di stato /fault_injector/status
  spike_duration    (float, default 0.05)     durata dello spike in secondi (solo fault_type=spike)

Esempio di utilizzo
--------------------
  # Terminale 1: avvio nodo
  ros2 run <pkg> fault_injector --ros-args -p channel:=2 -p fault_type:=offset -p fault_magnitude:=3.0

  # Terminale 2: attivazione a runtime
  ros2 param set /fault_injector fault_active true

  # Terminale 3: cambio tipo a runtime
  ros2 param set /fault_injector fault_type freeze

  # Terminale 4: disattivazione
  ros2 param set /fault_injector fault_active false

  # Monitoraggio stato
  ros2 topic echo /fault_injector/status
"""

import time
import numpy as np
import rclpy

from rclpy.node import Node
from std_msgs.msg import Float64, Float64MultiArray
from sensor_msgs.msg import JointState


# ============================================================
# Mappa fault_type (stringa) → ID numerico per il topic status
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
# Descrizione dei canali
# ============================================================
CHANNEL_INFO = {
    0: {
        'name': 'tau_ext_theta',
        'sub_topic': '/exo_dynamics/tau_ext_theta',
        'pub_topic': '/exo_dynamics/tau_ext_theta_faulted',
        'msg_type': 'Float64',
        'description': 'Forza utente proiettata su theta (sensore forza)',
    },
    1: {
        'name': 'trajectory_ref',
        'sub_topic': '/trajectory_ref',
        'pub_topic': '/trajectory_ref_faulted',
        'msg_type': 'Float64MultiArray',
        'description': 'Riferimento traiettoria [theta_ref, theta_dot_ref, theta_ddot_ref]',
    },
    2: {
        'name': 'torque',
        'sub_topic': '/torque',
        'pub_topic': '/torque_faulted',
        'msg_type': 'Float64',
        'description': 'Coppia attuata (inner loop / dito)',
    },
    3: {
        'name': 'joint_states',
        'sub_topic': '/joint_states',
        'pub_topic': '/joint_states_faulted',
        'msg_type': 'JointState',
        'description': 'Sensore di posizione/velocità encoder (ramo di retroazione)',
    },
}


class FaultInjector(Node):

    def __init__(self):
        super().__init__('fault_injector')

        # ============================================================
        # PARAMETRI
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
                f"Canale {channel} non valido. Scegliere tra {list(CHANNEL_INFO.keys())}."
            )
            raise ValueError(f"Canale non valido: {channel}")

        self._channel = channel
        self._ch_info = CHANNEL_INFO[channel]

        # ============================================================
        # STATO INTERNO
        # ============================================================
        self._frozen_value = None
        self._freeze_logged = False
        self._n_injections = 0
        self._last_raw = 0.0
        self._last_faulted = 0.0

        self._spike_active = False
        self._spike_start_time = None

        # ============================================================
        # ROS I/O — subscriber e publisher dipendenti dal canale
        # ============================================================
        if self._ch_info['msg_type'] == 'Float64':
            self._sub = self.create_subscription(
                Float64,
                self._ch_info['sub_topic'],
                self._cb_float64,
                10
            )
            self._pub = self.create_publisher(
                Float64,
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

        else:
            self._sub = self.create_subscription(
                Float64MultiArray,
                self._ch_info['sub_topic'],
                self._cb_multiarray,
                10
            )
            self._pub = self.create_publisher(
                Float64MultiArray,
                self._ch_info['pub_topic'],
                10
            )

        self._pub_status = self.create_publisher(
            Float64MultiArray,
            '/fault_injector/status',
            10
        )

        self._timer_status = self.create_timer(
            1.0 / publish_rate,
            self._publish_status
        )

        self.get_logger().info(
            f"FaultInjector avviato\n"
            f"  Canale          : {channel} — {self._ch_info['description']}\n"
            f"  Sub topic       : {self._ch_info['sub_topic']}\n"
            f"  Pub topic       : {self._ch_info['pub_topic']}\n"
            f"  fault_active    : {self.get_parameter('fault_active').value}\n"
            f"  fault_type      : {self.get_parameter('fault_type').value}\n"
            f"  fault_magnitude : {self.get_parameter('fault_magnitude').value}\n"
            f"\nPer attivare a runtime:\n"
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
        Freeze 'hard': il nodo continua a ricevere ma NON pubblica nulla
        sul topic faultato.
        """
        fault_active, fault_type, _, _, _ = self._current_fault_config()
        return fault_active and fault_type == 'freeze'

    def _reset_transient_states_if_needed(self):
        """
        Se freeze/spike non sono attivi, resetta lo stato interno relativo.
        Utile quando si cambia fault_type a runtime.
        """
        fault_active, fault_type, _, _, _ = self._current_fault_config()

        if (not fault_active) or (fault_type != 'freeze'):
            self._frozen_value = None
            self._freeze_logged = False

        if (not fault_active) or (fault_type != 'spike'):
            self._spike_active = False
            self._spike_start_time = None

    # ============================================================
    # CALLBACKS — ricezione messaggi
    # ============================================================

    def _cb_float64(self, msg: Float64):
        """
        Callback per canali Float64 (tau_ext_theta, torque).
        Se freeze è attivo, il nodo non pubblica nulla.
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
                    f"[FREEZE] Publishing bloccato su {self._ch_info['pub_topic']}. "
                    f"Ultimo valore osservato: {raw_val:.6f}"
                )
                self._freeze_logged = True

            self._n_injections += 1
            return

        faulted_val = self._apply_fault_scalar(raw_val)
        self._last_faulted = faulted_val

        out = Float64()
        out.data = faulted_val
        self._pub.publish(out)

    def _cb_multiarray(self, msg: Float64MultiArray):
        """
        Callback per canale Float64MultiArray (trajectory_ref).
        Se freeze è attivo, il nodo non pubblica nulla.
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
                    f"[FREEZE] Publishing bloccato su {self._ch_info['pub_topic']}. "
                    f"Ultimo valore osservato: {raw_val:.6f}"
                )
                self._freeze_logged = True

            self._n_injections += 1
            return

        faulted_val = self._apply_fault_scalar(raw_val)
        self._last_faulted = faulted_val

        data[target_idx] = faulted_val

        out = Float64MultiArray()
        out.data = data
        self._pub.publish(out)

    def _cb_joint_states(self, msg: JointState):
        """
        Callback per canale 3 — JointState.
        Se freeze è attivo, il nodo non pubblica nulla.
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
                    f"[FREEZE] Publishing bloccato su {self._ch_info['pub_topic']} "
                    f"per joint '{joint_name}'."
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
        Applica la fault al valore scalare in ingresso.
        Restituisce il valore modificato.
        Nota: il caso freeze è gestito nelle callback come blocco totale
        della pubblicazione, quindi qui non viene più usato.
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
                    f"[SPIKE] Impulso iniettato: {fault_magnitude:.4f}, "
                    f"durata {spike_duration:.3f} s"
                )

            elapsed = now - self._spike_start_time
            if elapsed <= spike_duration:
                self._n_injections += 1
                return value + fault_magnitude
            else:
                return value

        elif fault_type == 'freeze':
            # Non dovrebbe arrivarci, perché freeze è gestito nelle callback.
            return value

        else:
            self.get_logger().warn(
                f"fault_type '{fault_type}' non riconosciuto. "
                f"Valori validi: none, offset, noise, freeze, scale, spike"
            )
            return value

    # ============================================================
    # PUBBLICAZIONE STATO
    # ============================================================

    def _publish_status(self):
        """
        Pubblica lo stato del fault injector su /fault_injector/status.

        Layout Float64MultiArray:
          [0]  channel
          [1]  fault_type_id
          [2]  fault_active
          [3]  fault_magnitude
          [4]  n_injections
          [5]  last_raw
          [6]  last_faulted
          [7]  delta = last_faulted - last_raw
        """
        fault_active, fault_type, fault_magnitude, _, _ = self._current_fault_config()
        fault_type_id = float(FAULT_TYPE_MAP.get(fault_type, -1))

        delta = self._last_faulted - self._last_raw

        msg = Float64MultiArray()
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