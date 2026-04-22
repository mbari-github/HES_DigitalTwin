"""
koopman_fdi_testing.launch.py — Test del sistema Koopman FDI
=============================================================

Avvia lo stack completo senza bridge e safety manager:
  Plant + Admittance + Trajectory controller + Fault Injector
  + Koopman FDI node + Logger + RViz

Workflow consigliato:
  1. Lanciare con FAULT_ACTIVE = False
  2. Attendere che il nodo Koopman FDI raccolga 30s di dati healthy
     e completi il training (monitorare /koopman_fdi/status)
  3. Attivare il fault a runtime:
       ros2 param set /fault_injector fault_active true
  4. Osservare i residui:
       ros2 topic echo /koopman_fdi/fault_detected
       ros2 topic echo /koopman_fdi/residuals
  5. Disattivare il fault:
       ros2 param set /fault_injector fault_active false

Per cambiare canale di fault, modificare FAULT_CHANNEL in testa al file
e rilanciare (i remapping dipendono dal canale).

Nota: il nodo Koopman FDI legge i topic FAULTED (come farebbe un
vero sistema FDI nel loop reale). I segnali corrotti arrivano ai
controller e al nodo FDI, che deve rilevare la discrepanza.

Usage:
  ros2 launch exoskeletron_bringup koopman_fdi_testing.launch.py
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():

    description_pkg = get_package_share_directory('exoskeletron_description')
    bringup_pkg = get_package_share_directory('exoskeletron_bringup')

    urdf_file = os.path.join(description_pkg, 'urdf', 'assembly_with_hand.urdf')
    with open(urdf_file, 'r') as infp:
        robot_description_content = infp.read()

    dynamics_params_file = os.path.join(bringup_pkg, 'config', 'dynamics_params.yaml')
    koopman_params_file = os.path.join(bringup_pkg, 'config', 'koopman_fdi_params.yaml')

    # ============================================================
    # FAULT INJECTION CONFIGURATION
    # ============================================================
    # channel:
    #   0 → /exo_dynamics/tau_ext_theta   (force sensor)
    #   1 → /trajectory_ref               (admittance output)
    #   2 → /torque                       (actuator torque)
    #   3 → /joint_states                 (encoder)

    FAULT_CHANNEL       = 0
    FAULT_TYPE          = 'offset'
    FAULT_MAGNITUDE     = 2.0
    FAULT_ACTIVE        = False      # Attivare a runtime dopo il training
    NOISE_STD           = 0.1
    SPIKE_DURATION      = 0.2
    TARGET_INDEX        = 0
    FAULT_JS_FIELD      = 'position'
    JOINT_NAME          = 'rev_crank'

    # ============================================================
    # FAULT INJECTOR REMAPS
    # ============================================================
    # Schema: [producer] → /topic_raw → [FaultInjector] → /topic_faulted → [consumer]
    fault_injector_remaps = {
        0: [
            ('/exo_dynamics/tau_ext_theta', '/exo_dynamics/tau_ext_theta_raw'),
            ('/exo_dynamics/tau_ext_theta_faulted', '/exo_dynamics/tau_ext_theta_faulted'),
        ],
        1: [
            ('/trajectory_ref', '/trajectory_ref_raw'),
            ('/trajectory_ref_faulted', '/trajectory_ref_faulted'),
        ],
        2: [
            ('/torque', '/torque_raw'),
            ('/torque_faulted', '/torque_faulted'),
        ],
        3: [
            ('/joint_states', '/joint_states'),
            ('/joint_states_faulted', '/joint_states_faulted'),
        ],
    }

    # ============================================================
    # DYNAMICS NODE REMAPS
    # ============================================================
    dynamics_remaps = []
    if FAULT_CHANNEL == 0:
        # Dynamics publishes tau_ext on the _raw topic;
        # fault injector reads _raw and publishes _faulted
        dynamics_remaps.append(
            ('/exo_dynamics/tau_ext_theta', '/exo_dynamics/tau_ext_theta_raw')
        )
    if FAULT_CHANNEL == 2:
        # Dynamics reads faulted torque
        dynamics_remaps.append(
            ('/torque', '/torque_faulted')
        )

    # ============================================================
    # ADMITTANCE CONTROLLER REMAPS
    # ============================================================
    admittance_remaps = []
    if FAULT_CHANNEL == 0:
        # Admittance reads faulted tau_ext
        admittance_remaps.append(
            ('/exo_dynamics/tau_ext_theta', '/exo_dynamics/tau_ext_theta_faulted')
        )
    if FAULT_CHANNEL == 1:
        # Admittance publishes on _raw; fault injector reads _raw → _faulted
        admittance_remaps.append(
            ('/trajectory_ref', '/trajectory_ref_raw')
        )
    if FAULT_CHANNEL == 3:
        admittance_remaps.append(
            ('/joint_states', '/joint_states_faulted')
        )

    # ============================================================
    # TRAJECTORY CONTROLLER REMAPS
    # ============================================================
    trajectory_remaps = []
    if FAULT_CHANNEL == 1:
        # Trajectory controller reads faulted reference
        trajectory_remaps.append(
            ('/trajectory_ref', '/trajectory_ref_faulted')
        )
    if FAULT_CHANNEL == 2:
        # Trajectory controller publishes on _raw
        trajectory_remaps.append(
            ('/torque', '/torque_raw')
        )
    if FAULT_CHANNEL == 3:
        trajectory_remaps.append(
            ('/joint_states', '/joint_states_faulted')
        )

    # ============================================================
    # RVIZ REMAPS
    # ============================================================
    rviz_remaps = []
    if FAULT_CHANNEL == 3:
        rviz_remaps.append(
            ('/joint_states', '/joint_states_faulted')
        )

    # ============================================================
    # KOOPMAN FDI NODE REMAPS
    # ============================================================
    # The FDI node must read the FAULTED signals (it sits in the
    # real signal path, like a real FDI system would).
    # For the input signals (τ_m, τ_ext), it reads what the plant
    # actually receives.
    koopman_remaps = []
    if FAULT_CHANNEL == 0:
        # FDI sees the faulted tau_ext (same as admittance controller)
        koopman_remaps.append(
            ('/exo_dynamics/tau_ext_theta', '/exo_dynamics/tau_ext_theta_faulted')
        )
    if FAULT_CHANNEL == 1:
        # FDI reads the faulted trajectory reference
        koopman_remaps.append(
            ('/trajectory_ref', '/trajectory_ref_faulted')
        )
    if FAULT_CHANNEL == 2:
        # FDI reads the faulted torque (what the plant actually sees)
        koopman_remaps.append(
            ('/torque', '/torque_faulted')
        )
    if FAULT_CHANNEL == 3:
        # FDI reads faulted joint states
        koopman_remaps.append(
            ('/joint_states', '/joint_states_faulted')
        )

    # ============================================================
    # NODES
    # ============================================================

    return LaunchDescription([

        # ── Robot State Publisher ────────────────────────────────
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot_description_content}],
        ),

        # ── Dynamic Plant ────────────────────────────────────────
        Node(
            package='exoskeletron_dynamics',
            executable='exo_dynamics',
            name='dynamics',
            output='screen',
            parameters=[dynamics_params_file],
            remappings=dynamics_remaps,
        ),

        # ── RViz ─────────────────────────────────────────────────
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', os.path.join(
                description_pkg, 'rviz', 'exo_display.rviz')],
            remappings=rviz_remaps,
        ),

        # ── External Wrench Publisher (input source) ─────────────
        Node(
            package='exoskeletron_utils',
            executable='external_wrench_pub',
            name='input',
            output='screen',
        ),

        # ── Admittance Controller ────────────────────────────────
        Node(
            package='exoskeletron_control',
            executable='admittance_controller',
            name='admittance_controller',
            output='screen',
            remappings=admittance_remaps,
        ),

        # ── Trajectory Controller ────────────────────────────────
        Node(
            package='exoskeletron_control',
            executable='trajectory_controller',
            name='trajectory_controller',
            output='screen',
            remappings=trajectory_remaps,
        ),

        # ── Fault Injector ───────────────────────────────────────
        Node(
            package='exoskeletron_faults',
            executable='fault_injector',
            name='fault_injector',
            output='screen',
            parameters=[{
                'channel':          FAULT_CHANNEL,
                'fault_active':     FAULT_ACTIVE,
                'fault_type':       FAULT_TYPE,
                'fault_magnitude':  FAULT_MAGNITUDE,
                'noise_std':        NOISE_STD,
                'spike_duration':   SPIKE_DURATION,
                'target_index':     TARGET_INDEX,
                'fault_js_field':   FAULT_JS_FIELD,
                'joint_name':       JOINT_NAME,
                'publish_rate':     200.0,
            }],
            remappings=fault_injector_remaps[FAULT_CHANNEL],
        ),

        # ── Koopman FDI Node ─────────────────────────────────────
        Node(
            package='exoskeletron_observers',
            executable='koopman_fdi_node',
            name='koopman_fdi_node',
            output='screen',
            parameters=[koopman_params_file],
            remappings=koopman_remaps,
        ),

        # ── CSV Logger ───────────────────────────────────────────
        Node(
            package='exoskeletron_utils',
            executable='logger',
            name='exo_logger',
            output='screen',
            parameters=[{
                'log_rate': 200.0,
            }],
        ),
    ])