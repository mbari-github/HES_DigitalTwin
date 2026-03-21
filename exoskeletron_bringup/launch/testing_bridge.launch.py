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
    # CONFIGURAZIONE GENERALE
    # ============================================================
    USE_FAULT_INJECTOR = True

    # ============================================================
    # CONFIGURAZIONE FAULT INJECTION
    # ============================================================
    # channel:
    #   0 -> /exo_dynamics/tau_ext_theta
    #   1 -> /trajectory_ref
    #   2 -> /torque
    #   3 -> /joint_states
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

    FAULT_CHANNEL       = 0
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
    # - con fault injector e channel 0:
    #     pubblica /exo_dynamics/tau_ext_theta_raw
    # - senza fault injector:
    #     pubblica il topic nominale /exo_dynamics/tau_ext_theta
    # - legge sempre /torque finale dal bridge
    # ------------------------------------------------------------
    dynamics_remaps = []
    if USE_FAULT_INJECTOR and FAULT_CHANNEL == 0:
        dynamics_remaps.append(
            ('/exo_dynamics/tau_ext_theta', '/exo_dynamics/tau_ext_theta_raw')
        )

    # ------------------------------------------------------------
    # admittance controller
    #
    # senza injector:
    #   legge /exo_dynamics/tau_ext_theta
    #   legge /joint_states
    #   pubblica /trajectory_ref_raw
    #
    # con injector:
    #   continua a pubblicare /trajectory_ref_raw
    #   se ch 0 legge /exo_dynamics/tau_ext_theta_faulted
    #   se ch 3 legge /joint_states_faulted
    # ------------------------------------------------------------
    admittance_remaps = [
        ('/trajectory_ref', '/trajectory_ref_raw'),
    ]

    if USE_FAULT_INJECTOR:
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
    #
    # senza injector:
    #   legge /trajectory_ref dal bridge
    #   legge /joint_states
    #   pubblica /torque_raw
    #
    # con injector:
    #   legge /trajectory_ref dal bridge
    #   pubblica /torque_raw
    #   se ch 3 legge /joint_states_faulted
    # ------------------------------------------------------------
    trajectory_remaps = [
        ('/torque', '/torque_raw'),
    ]

    if USE_FAULT_INJECTOR and FAULT_CHANNEL == 3:
        trajectory_remaps.append(
            ('/joint_states', '/joint_states_faulted')
        )

    # ------------------------------------------------------------
    # rviz
    # senza injector: legge /joint_states
    # con injector e ch 3: legge /joint_states_faulted
    # ------------------------------------------------------------
    rviz_remaps = []
    if USE_FAULT_INJECTOR and FAULT_CHANNEL == 3:
        rviz_remaps.append(
            ('/joint_states', '/joint_states_faulted')
        )

    # ------------------------------------------------------------
    # exo_bridge
    #
    # senza injector:
    #   legge:
    #       /trajectory_ref_raw
    #       /torque_raw
    #       /joint_states
    #       /exo_dynamics/tau_ext_theta
    #
    # con injector:
    #   ch 0 -> legge tau_ext_theta_faulted
    #   ch 1 -> legge trajectory_ref_faulted al posto di trajectory_ref_raw
    #   ch 2 -> legge torque_faulted al posto di torque_raw
    #   ch 3 -> legge joint_states_faulted
    # ------------------------------------------------------------
    bridge_remaps = []

    if USE_FAULT_INJECTOR:
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

    # ------------------------------------------------------------
    # Lista nodi
    # ------------------------------------------------------------
    nodes = [

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
        # Exo Bridge
        # --------------------------------------------------------
        Node(
            package='exoskeletron_supervision',
            executable='exo_bridge',
            name='exo_bridge',
            output='screen',
            parameters=[{
                'publish_rate_hz': 200.0,
                # 'override_tau_limit': 10.0,
                # 'compliant_tau_limit': 10.0,
                # 'compliant_vel_limit': 10.0,
                # 'compliant_acc_limit': 10.0,
                'stop_mode': 'hold_only',
            }],
            remappings=bridge_remaps
        ),
    ]

    # ------------------------------------------------------------
    # Fault Injector opzionale
    # ------------------------------------------------------------
    if USE_FAULT_INJECTOR:
        nodes.append(
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
            )
        )

    return LaunchDescription(nodes)