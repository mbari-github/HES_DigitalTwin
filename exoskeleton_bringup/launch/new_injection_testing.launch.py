"""
injection_testing_framework.launch.py
======================================

Launch file per il test del FaultFramework (pipeline-based fault injection).
Basato su injection_testing.launch.py, usa fault_framework.py invece di
fault_injector.py e carica la pipeline dal file YAML di configurazione.

Differenze rispetto a injection_testing.launch.py:
  - Executable: 'fault_framework' invece di 'fault_injector'
  - I parametri del fault sono caricati da fault_framework_params.yaml
  - La pipeline è configurata nel YAML (bias_drift + noise + quantization)
  - fault_active è False all'avvio: attivare a runtime con:
      ros2 param set /fault_injector fault_active true

Canali supportati:
  0 → /exo_dynamics/tau_ext_theta  (load cell / forza esterna)
  3 → /joint_states                (encoder posizione/velocità)

Flusso tipico:
  1. Lanciare il file
  2. Attendere che il sistema si stabilizzi (~5s)
  3. Attivare il fault:
       ros2 param set /fault_injector fault_active true
  4. Monitorare i residui degli observer:
       ros2 topic echo /observer/momentum_residual
       ros2 topic echo /observer/state_residual
  5. Monitorare lo status del fault injector:
       ros2 topic echo /fault_injector/status
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():

    # ── Package paths ────────────────────────────────────────────────
    description_pkg = get_package_share_directory('exoskeleton_description')
    bringup_pkg     = get_package_share_directory('exoskeleton_bringup')

    # ── URDF ─────────────────────────────────────────────────────────
    urdf_file = os.path.join(
        description_pkg, 'urdf', 'assembly_with_hand.urdf'
    )
    with open(urdf_file, 'r') as f:
        robot_description_content = f.read()

    # ── Config files ─────────────────────────────────────────────────
    observer_params_file       = os.path.join(bringup_pkg, 'config', 'observer_params.yaml')
    fault_framework_params_file = os.path.join(bringup_pkg, 'config', 'fault_framework_params.yaml')

    # ════════════════════════════════════════════════════════════════
    # FAULT CONFIGURATION
    # ════════════════════════════════════════════════════════════════
    # Canale iniettato — modificare qui per cambiare canale:
    #   0 → tau_ext_theta  (sensore forza / load cell)
    #   3 → joint_states   (encoder posizione/velocità)
    #
    # Il tipo di fault e la pipeline sono definiti in:
    #   exoskeleton_bringup/config/fault_framework_params.yaml
    #
    # fault_active è False all'avvio per permettere al sistema di
    # stabilizzarsi prima di iniettare il fault.

    FAULT_CHANNEL = 0   # deve corrispondere al 'channel' nel YAML

    # ════════════════════════════════════════════════════════════════
    # REMAPPING LOGIC
    # ════════════════════════════════════════════════════════════════

    # ── Fault framework remaps ───────────────────────────────────────
    # Il nodo fault_framework si inserisce come proxy:
    #   [produttore] → /topic_raw → [fault_framework] → /topic_faulted → [consumatori]
    fault_framework_remaps = {
        0: [
            ('/exo_dynamics/tau_ext_theta',         '/exo_dynamics/tau_ext_theta_raw'),
            ('/exo_dynamics/tau_ext_theta_faulted',  '/exo_dynamics/tau_ext_theta_faulted'),
        ],
        3: [
            ('/joint_states',         '/joint_states'),
            ('/joint_states_faulted', '/joint_states_faulted'),
        ],
    }

    # ── Dynamics node ────────────────────────────────────────────────
    # Ch0: il dynamics pubblica tau_ext su _raw, il framework la intercetta
    # Ch3: nessuna modifica necessaria al dynamics
    dynamics_remaps = []
    if FAULT_CHANNEL == 0:
        dynamics_remaps.append(
            ('/exo_dynamics/tau_ext_theta', '/exo_dynamics/tau_ext_theta_raw')
        )

    # ── Admittance controller ────────────────────────────────────────
    # L'admittance deve leggere il segnale faultato per propagare
    # l'effetto nella closed-loop come avviene nel sistema reale.
    admittance_remaps = []
    if FAULT_CHANNEL == 0:
        admittance_remaps.append(
            ('/exo_dynamics/tau_ext_theta', '/exo_dynamics/tau_ext_theta_faulted')
        )
    if FAULT_CHANNEL == 3:
        admittance_remaps.append(
            ('/joint_states', '/joint_states_faulted')
        )

    # ── Trajectory controller ────────────────────────────────────────
    trajectory_remaps = []
    if FAULT_CHANNEL == 3:
        trajectory_remaps.append(
            ('/joint_states', '/joint_states_faulted')
        )

    # ── RViz ─────────────────────────────────────────────────────────
    rviz_remaps = []
    if FAULT_CHANNEL == 3:
        rviz_remaps.append(
            ('/joint_states', '/joint_states_faulted')
        )

    # ── Observer ────────────────────────────────────────────────────
    # L'observer deve vedere gli stessi segnali che vede il controller,
    # così i residui riflettono il fault effettivamente iniettato.
    # /torque_raw: l'observer legge la coppia pre-bridge
    observer_remaps = []
    if FAULT_CHANNEL == 0:
        observer_remaps.append(
            ('/exo_dynamics/tau_ext_theta', '/exo_dynamics/tau_ext_theta_faulted')
        )
    if FAULT_CHANNEL == 3:
        observer_remaps.append(
            ('/joint_states', '/joint_states_faulted')
        )
    observer_remaps.append(
        ('/torque_raw', '/torque')
    )

    # ════════════════════════════════════════════════════════════════
    # NODES
    # ════════════════════════════════════════════════════════════════

    return LaunchDescription([

        # ── Robot State Publisher ─────────────────────────────────────
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot_description_content}],
        ),

        # ── Plant dynamics ────────────────────────────────────────────
        Node(
            package='exoskeleton_dynamics',
            executable='exo_dynamics',
            name='dynamics',
            output='screen',
            remappings=dynamics_remaps,
        ),

        # ── RViz ─────────────────────────────────────────────────────
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=[
                '-d',
                os.path.join(description_pkg, 'rviz', 'exo_display.rviz'),
            ],
            remappings=rviz_remaps,
        ),

        # ── External input (wrench sinusoidale) ───────────────────────
        # Eccita la dinamica con una forza periodica durante il test.
        # Parametri di default: f=0.05 Hz, tau_min=-12 N, tau_max=0 N
        Node(
            package='exoskeleton_utils',
            executable='external_wrench_pub',
            name='input',
            output='screen',
        ),

        # ── Admittance Controller (outer loop) ────────────────────────
        Node(
            package='exoskeleton_control',
            executable='admittance_controller',
            name='admittance_controller',
            output='screen',
            parameters=[{'force_deadband': 0.0}],
            remappings=admittance_remaps,
        ),

        # ── Trajectory Controller (inner loop) ────────────────────────
        Node(
            package='exoskeleton_control',
            executable='trajectory_controller',
            name='trajectory_controller',
            output='screen',
            remappings=trajectory_remaps,
        ),

        # ── Observer (model-based fault detection) ────────────────────
        Node(
            package='exoskeleton_observers',
            executable='observer_node',
            name='observer_node',
            output='screen',
            parameters=[observer_params_file],
            remappings=observer_remaps,
        ),

        # ── Fault Framework ───────────────────────────────────────────
        # Pipeline configurata in fault_framework_params.yaml:
        #   bias_drift → noise → quantization (su channel 0, torque)
        #
        # Il nodo si chiama ancora 'fault_injector' per compatibilità
        # con i tool di monitoring esistenti (ros2 topic echo /fault_injector/status)
        Node(
            package='exoskeleton_faults',
            executable='fault_framework',
            name='fault_injector',
            output='screen',
            parameters=[fault_framework_params_file],
            remappings=fault_framework_remaps[FAULT_CHANNEL],
        ),

    ])