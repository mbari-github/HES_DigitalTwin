"""
neural_fdi_testing_framework.launch.py
========================================

Launch file per il test del sistema Neural FDI con pipeline-based
fault injection (FaultFramework invece di FaultInjector).

Basato su neural_fdi_testing.launch.py, sostituisce il nodo
'fault_injector' con 'fault_framework' e carica la pipeline
dal file fault_framework_params.yaml.

Differenze rispetto a neural_fdi_testing.launch.py:
  - Executable: 'fault_framework' invece di 'fault_injector'
  - Parametri fault caricati da fault_framework_params.yaml
  - Parametri Neural FDI (skip_collect, skip_train, collect_duration)
    caricati esclusivamente da neural_fdi_params.yaml
  - Nessun parametro hardcoded nel launch: tutto nei YAML

Canali supportati:
  0 → /exo_dynamics/tau_ext_theta  (load cell)
  3 → /joint_states                (encoder)

Flusso tipico:
  1. Configurare neural_fdi_params.yaml:
       skip_collect: false
       skip_train:   false
       collect_duration: 60.0

  2. Lanciare con fault_active: false nel fault_framework_params.yaml
     → Il nodo FDI raccoglie dati healthy, addestra le reti,
       e passa automaticamente a INFERENCE.

  3. Una volta in INFERENCE, attivare il fault a runtime:
       ros2 param set /fault_injector fault_active true

  4. Monitorare diagnosi e delta:
       ros2 topic echo /neural_fdi/diagnosis
       ros2 topic echo /fault_injector/status

  5. Cambiare pipeline a runtime senza riavviare:
       ros2 param set /fault_injector fault_pipeline_config \
         '[{"type":"offset","magnitude":0.5}]'

  6. Disattivare:
       ros2 param set /fault_injector fault_active false
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():

    # ── Package paths ────────────────────────────────────────────────
    description_pkg = get_package_share_directory('exoskeleton_description')
    bringup_pkg     = get_package_share_directory('exoskeleton_bringup')
    neural_fdi_pkg  = get_package_share_directory('exoskeleton_neural_fdi')

    # ── URDF ─────────────────────────────────────────────────────────
    urdf_file = os.path.join(
        description_pkg, 'urdf', 'assembly_with_hand.urdf'
    )
    with open(urdf_file, 'r') as f:
        robot_description_content = f.read()

    # ── Config files ─────────────────────────────────────────────────
    # Tutti i parametri operativi sono nei YAML.
    # Il launch non sovrascrive nessun valore.
    neural_fdi_params_file      = os.path.join(
        neural_fdi_pkg, 'config', 'neural_fdi_params.yaml'
    )
    fault_framework_params_file = os.path.join(
        bringup_pkg, 'config', 'fault_framework_params.yaml'
    )

    # ════════════════════════════════════════════════════════════════
    # FAULT CHANNEL
    # ════════════════════════════════════════════════════════════════
    #
    #   0 → /exo_dynamics/tau_ext_theta  (load cell / forza esterna)
    #   3 → /joint_states                (encoder posizione/velocità)

    FAULT_CHANNEL = 0

    # ════════════════════════════════════════════════════════════════
    # REMAPPING LOGIC
    # ════════════════════════════════════════════════════════════════

    # ── Fault framework remaps ───────────────────────────────────────
    # Schema proxy:
    #   [produttore] → /topic_raw → [fault_framework] → /topic_faulted → [consumatori]
    fault_framework_remaps = {
        0: [
            ('/exo_dynamics/tau_ext_theta',
             '/exo_dynamics/tau_ext_theta_raw'),
            ('/exo_dynamics/tau_ext_theta_faulted',
             '/exo_dynamics/tau_ext_theta_faulted'),
        ],
        3: [
            ('/joint_states',
             '/joint_states'),
            ('/joint_states_faulted',
             '/joint_states_faulted'),
        ],
    }

    # ── Dynamics ─────────────────────────────────────────────────────
    dynamics_remaps = []
    if FAULT_CHANNEL == 0:
        dynamics_remaps.append(
            ('/exo_dynamics/tau_ext_theta',
             '/exo_dynamics/tau_ext_theta_raw')
        )

    # ── Admittance controller ────────────────────────────────────────
    admittance_remaps = []
    if FAULT_CHANNEL == 0:
        admittance_remaps.append(
            ('/exo_dynamics/tau_ext_theta',
             '/exo_dynamics/tau_ext_theta_faulted')
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

    # ── Neural FDI ───────────────────────────────────────────────────
    neural_fdi_remaps = []
    if FAULT_CHANNEL == 0:
        neural_fdi_remaps.append(
            ('/exo_dynamics/tau_ext_theta',
             '/exo_dynamics/tau_ext_theta_faulted')
        )
    if FAULT_CHANNEL == 3:
        neural_fdi_remaps.append(
            ('/joint_states', '/joint_states_faulted')
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
        Node(
            package='exoskeleton_utils',
            executable='external_wrench_pub',
            name='input',
            output='screen',
            parameters=[{
                'frequency': 0.05,
                'f_min':    -12.0,
                'f_max':     0.0,
            }],
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

        # ── Fault Framework ───────────────────────────────────────────
        Node(
            package='exoskeleton_faults',
            executable='fault_framework',
            name='fault_injector',
            output='screen',
            parameters=[fault_framework_params_file],
            remappings=fault_framework_remaps[FAULT_CHANNEL],
        ),

        # ── Neural FDI Node ───────────────────────────────────────────
        Node(
            package='exoskeleton_neural_fdi',
            executable='fdi_node',
            name='neural_fdi_node',
            output='screen',
            parameters=[neural_fdi_params_file],
            remappings=neural_fdi_remaps,
        ),

    ])