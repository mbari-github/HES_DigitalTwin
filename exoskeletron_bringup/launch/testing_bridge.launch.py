from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():

    description_pkg = get_package_share_directory('exoskeletron_description')
    urdf_file = os.path.join(description_pkg, 'urdf', 'assembly_with_hand.urdf')

    with open(urdf_file, 'r') as infp:
        robot_description_content = infp.read()

    # ============================================================
    # CONFIGURAZIONE FAULT INJECTION
    # ============================================================
    # channel:
    #   0 → /exo_dynamics/tau_ext_theta
    #   1 → /trajectory_ref
    #   2 → /torque
    #   3 → /joint_states
    #
    # Con bridge:
    #
    # ch 0:
    #   /exo_dynamics/tau_ext_theta_raw
    #       -> /exo_dynamics/tau_ext_theta_faulted
    #       -> letto da admittance_controller e bridge
    #
    # ch 1:
    #   /trajectory_ref_raw
    #       -> /trajectory_ref_faulted
    #       -> letto da exo_bridge
    #       -> pubblicato finale su /trajectory_ref
    #
    # ch 2:
    #   /torque_raw
    #       -> /torque_faulted
    #       -> letto da exo_bridge
    #       -> pubblicato finale su /torque
    #
    # ch 3:
    #   /joint_states
    #       -> /joint_states_faulted
    #       -> letto da controllori, rviz e bridge
    # ============================================================

    FAULT_CHANNEL       = 3
    FAULT_TYPE          = 'freeze'
    FAULT_MAGNITUDE     = 0.0
    FAULT_ACTIVE        = False
    NOISE_STD           = 0.1
    SPIKE_DURATION      = 0.05
    TARGET_INDEX        = 0
    FAULT_JS_FIELD      = 'position'
    JOINT_NAME          = 'rev_crank'

    # ------------------------------------------------------------
    # fault injector remaps
    # ------------------------------------------------------------
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

    # ------------------------------------------------------------
    # dynamics
    # - pubblica tau_ext su raw se fault channel 0
    # - legge sempre /torque finale dal bridge
    # ------------------------------------------------------------
    dynamics_remaps = []
    if FAULT_CHANNEL == 0:
        dynamics_remaps.append(
            ('/exo_dynamics/tau_ext_theta', '/exo_dynamics/tau_ext_theta_raw')
        )

    # ------------------------------------------------------------
    # admittance controller
    # - pubblica SEMPRE /trajectory_ref_raw
    # - legge tau_ext faulted se ch 0
    # - legge joint_states faulted se ch 3
    # ------------------------------------------------------------
    admittance_remaps = [
        ('/trajectory_ref', '/trajectory_ref_raw'),
    ]
    if FAULT_CHANNEL == 0:
        admittance_remaps.append(
            ('/exo_dynamics/tau_ext_theta', '/exo_dynamics/tau_ext_theta_faulted')
        )
    if FAULT_CHANNEL == 3:
        admittance_remaps.append(
            ('/joint_states', '/joint_states_faulted')
        )

    # ------------------------------------------------------------
    # trajectory controller
    # - legge SEMPRE /trajectory_ref finale dal bridge
    # - pubblica SEMPRE /torque_raw
    # - legge joint_states faulted se ch 3
    # ------------------------------------------------------------
    trajectory_remaps = [
        ('/torque', '/torque_raw'),
    ]
    if FAULT_CHANNEL == 3:
        trajectory_remaps.append(
            ('/joint_states', '/joint_states_faulted')
        )

    # ------------------------------------------------------------
    # rviz
    # - legge joint_states faulted se ch 3
    # ------------------------------------------------------------
    rviz_remaps = []
    if FAULT_CHANNEL == 3:
        rviz_remaps.append(
            ('/joint_states', '/joint_states_faulted')
        )

    # ------------------------------------------------------------
    # exo_bridge
    #
    # di default legge:
    #   /trajectory_ref_raw
    #   /torque_raw
    #   /joint_states
    #   /exo_dynamics/tau_ext_theta
    #
    # se c'è fault su canale 1 o 2, gli facciamo leggere il FAULTED
    # al posto del RAW corrispondente.
    #
    # se c'è fault su canale 0 o 3, gli facciamo leggere i sensori faulted
    # per coerenza con il resto del sistema e per monitoraggio.
    # ------------------------------------------------------------
    bridge_remaps = []

    if FAULT_CHANNEL == 0:
        bridge_remaps.append(
            ('/exo_dynamics/tau_ext_theta', '/exo_dynamics/tau_ext_theta_faulted')
        )

    if FAULT_CHANNEL == 1:
        bridge_remaps.append(
            ('/trajectory_ref_raw', '/trajectory_ref_faulted')
        )

    if FAULT_CHANNEL == 2:
        bridge_remaps.append(
            ('/torque_raw', '/torque_faulted')
        )

    if FAULT_CHANNEL == 3:
        bridge_remaps.append(
            ('/joint_states', '/joint_states_faulted')
        )

    return LaunchDescription([

        # --------------------------------------------------------
        # Robot State Publisher
        # --------------------------------------------------------
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot_description_content}]
        ),

        # --------------------------------------------------------
        # Plant
        # --------------------------------------------------------
        Node(
            package='exoskeletron_dynamics',
            executable='exo_dynamics',
            name='dynamics',
            output='screen',
            remappings=dynamics_remaps
        ),

        # --------------------------------------------------------
        # RViz
        # --------------------------------------------------------
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', os.path.join(description_pkg, 'rviz', 'exo_display.rviz')],
            remappings=rviz_remaps
        ),

        # --------------------------------------------------------
        # Input su /exo_dynamics/external_wrench
        # --------------------------------------------------------
        Node(
            package='exoskeletron_utils',
            executable='external_wrench_pub',
            name='input',
            output='screen',
        ),

        # --------------------------------------------------------
        # Admittance Controller
        # --------------------------------------------------------
        Node(
            package='exoskeletron_control',
            executable='admittance_controller',
            name='admittance_controller',
            output='screen',
            remappings=admittance_remaps
        ),

        # --------------------------------------------------------
        # Trajectory Controller
        # --------------------------------------------------------
        Node(
            package='exoskeletron_control',
            executable='trajectory_controller',
            name='trajectory_controller',
            output='screen',
            remappings=trajectory_remaps
        ),

        # --------------------------------------------------------
        # Fault Injector
        # --------------------------------------------------------
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
            remappings=fault_injector_remaps[FAULT_CHANNEL]
        ),

        # --------------------------------------------------------
        # Exo Bridge
        # --------------------------------------------------------
        Node(
            package='exoskeletron_supervision',
            executable='exo_bridge',
            name='exo_bridge',
            output='screen',
            parameters=[{
            'publish_rate_hz': 200.0,

            # Valori per testare altre modalità senza innescare accidentalmente altre modalità
            #'override_tau_limit': 10.0,
            #'compliant_tau_limit': 10.0,
            #'compliant_vel_limit': 10.0,
            #'compliant_acc_limit': 10.0, 
            'stop_mode': 'hold_only',
            }],  
            remappings=bridge_remaps
        ),
    ])