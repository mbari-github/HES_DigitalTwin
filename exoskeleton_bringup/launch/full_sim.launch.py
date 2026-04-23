from launch import LaunchDescription
from launch.actions import TimerAction
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os



def generate_launch_description():

    description_pkg = get_package_share_directory('exoskeleton_description')
    launch_pkg      = get_package_share_directory('exoskeleton_bringup')

    urdf_file = os.path.join(description_pkg, 'urdf', 'assembly_with_hand.urdf')
    with open(urdf_file, 'r') as infp:
        robot_description_content = infp.read()

    safety_params_file     = os.path.join(launch_pkg, 'config', 'safety_params.yaml')
    exo_bridge_params_file = os.path.join(launch_pkg, 'config', 'exo_bridge_params.yaml')
    dynamics_params_file   = os.path.join(launch_pkg, 'config', 'dynamics_params.yaml')

    # ============================================================
    # GENERAL CONFIGURATION
    # ============================================================
    USE_FAULT_INJECTOR = True

    # ============================================================
    # FAULT INJECTION CONFIGURATION
    # ============================================================
    # channel:
    #   0 -> /exo_dynamics/tau_ext_theta
    #   1 -> /trajectory_ref
    #   2 -> /torque
    #   3 -> /joint_states

    FAULT_CHANNEL   = 3
    FAULT_TYPE      = 'offset'
    FAULT_MAGNITUDE = 0.1
    FAULT_ACTIVE    = False
    NOISE_STD       = 0.1
    SPIKE_DURATION  = 0.05
    TARGET_INDEX    = 0
    FAULT_JS_FIELD  = 'both'
    JOINT_NAME      = 'rev_crank'

    # ------------------------------------------------------------
    # Fault injector remaps — redirect original topics to raw/faulted variants
    # ------------------------------------------------------------
    fault_injector_remaps = {
        0: [
            ('/exo_dynamics/tau_ext_theta',         '/exo_dynamics/tau_ext_theta_raw'),
            ('/exo_dynamics/tau_ext_theta_faulted', '/exo_dynamics/tau_ext_theta_faulted'),
        ],
        1: [
            ('/trajectory_ref',          '/trajectory_ref_raw'),
            ('/trajectory_ref_faulted',  '/trajectory_ref_faulted'),
        ],
        2: [
            ('/torque',          '/torque_raw'),
            ('/torque_faulted',  '/torque_faulted'),
        ],
        3: [
            ('/joint_states',         '/joint_states'),
            ('/joint_states_faulted', '/joint_states_faulted'),
        ],
    }

    # ------------------------------------------------------------
    # Remaps dynamics
    # ------------------------------------------------------------
    dynamics_remaps = []
    if USE_FAULT_INJECTOR and FAULT_CHANNEL == 0:
        dynamics_remaps.append(
            ('/exo_dynamics/tau_ext_theta', '/exo_dynamics/tau_ext_theta_raw')
        )

    # ------------------------------------------------------------
    # Remaps admittance controller
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
    # Remaps trajectory controller
    # ------------------------------------------------------------
    trajectory_remaps = [
        ('/torque', '/torque_raw'),
    ]
    if USE_FAULT_INJECTOR and FAULT_CHANNEL == 3:
        trajectory_remaps.append(
            ('/joint_states', '/joint_states_faulted')
        )

    # ------------------------------------------------------------
    # Remaps RViz
    # ------------------------------------------------------------
    rviz_remaps = []
    if USE_FAULT_INJECTOR and FAULT_CHANNEL == 3:
        rviz_remaps.append(
            ('/joint_states', '/joint_states_faulted')
        )

    # ------------------------------------------------------------
    # Remaps exo_bridge
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

    # ============================================================
    # NODES — started immediately
    # ============================================================
    nodes_immediate = [

        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot_description_content}]
        ),

        Node(
            package='exoskeleton_dynamics',
            executable='exo_dynamics',
            name='dynamics',
            output='screen',
            parameters=[dynamics_params_file],
            remappings=dynamics_remaps
        ),

        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', os.path.join(description_pkg, 'rviz', 'exo_display.rviz')],
            remappings=rviz_remaps
        ),

        Node(
            package='exoskeleton_utils',
            executable='external_wrench_pub',
            name='input',
            output='screen',
        ),

        Node(
            package='exoskeleton_control',
            executable='admittance_controller',
            name='admittance_controller',
            output='screen',
            remappings=admittance_remaps
        ),

        Node(
            package='exoskeleton_control',
            executable='trajectory_controller',
            name='trajectory_controller',
            output='screen',
            remappings=trajectory_remaps
        ),

        Node(
            package='exoskeleton_supervision',
            executable='exo_bridge',
            name='exo_bridge',
            output='screen',
            parameters=[exo_bridge_params_file],
            remappings=bridge_remaps
        ),
    ]

    # ============================================================
    # NODES — delayed start (5 s) to let the bridge and dynamics
    # settle before the safety state machine begins monitoring
    # ============================================================
    nodes_delayed = [

        Node(
            package='exoskeleton_safety_manager',
            executable='state_machine_node',
            name='state_machine',
            output='screen',
            parameters=[safety_params_file],
        ),
    ]

    # ============================================================
    # FAULT INJECTOR NODE (conditional)
    # ============================================================
    if USE_FAULT_INJECTOR:
        nodes_immediate.append(
            Node(
                package='exoskeleton_faults',
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

    return LaunchDescription(
        nodes_immediate
        + [
            TimerAction(
                period=5.0,
                actions=nodes_delayed,
            )
        ]
    )