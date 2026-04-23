from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():

    description_pkg = get_package_share_directory('exoskeleton_description')
    launch_pkg = get_package_share_directory('exoskeleton_bringup')

    urdf_file = os.path.join(description_pkg, 'urdf', 'assembly_with_hand.urdf')

    bringup_pkg = get_package_share_directory('exoskeleton_bringup')
    observer_params_file = os.path.join(bringup_pkg, 'config', 'observer_params.yaml')

    with open(urdf_file, 'r') as infp:
        robot_description_content = infp.read()

    # ============================================================
    # FAULT INJECTION CONFIGURATION
    # ============================================================
    # channel:
    #   0 → /exo_dynamics/tau_ext_theta
    #   1 → /trajectory_ref
    #   2 → /torque
    #   3 → /joint_states

    FAULT_CHANNEL       = 0
    FAULT_TYPE          = 'spike'
    FAULT_MAGNITUDE     = -5.0
    FAULT_ACTIVE        = False
    NOISE_STD           = 0.1
    SPIKE_DURATION      = 0.2
    TARGET_INDEX        = 0
    FAULT_JS_FIELD      = 'position'
    JOINT_NAME          = 'rev_crank'

    # ------------------------------------------------------------
    # Remap fault injector
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
    # Dynamics node
    # ------------------------------------------------------------
    dynamics_remaps = []
    if FAULT_CHANNEL == 0:
        dynamics_remaps.append(
            ('/exo_dynamics/tau_ext_theta', '/exo_dynamics/tau_ext_theta_raw')
        )
    if FAULT_CHANNEL == 2:
        dynamics_remaps.append(
            ('/torque', '/torque_faulted')
        )

    # ------------------------------------------------------------
    # Admittance controller
    # ------------------------------------------------------------
    admittance_remaps = []
    if FAULT_CHANNEL == 0:
        admittance_remaps.append(
            ('/exo_dynamics/tau_ext_theta', '/exo_dynamics/tau_ext_theta_faulted')
        )
    if FAULT_CHANNEL == 1:
        admittance_remaps.append(
            ('/trajectory_ref', '/trajectory_ref_raw')
        )
    if FAULT_CHANNEL == 3:
        admittance_remaps.append(
            ('/joint_states', '/joint_states_faulted')
        )

    # ------------------------------------------------------------
    # Trajectory controller
    # ------------------------------------------------------------
    trajectory_remaps = []
    if FAULT_CHANNEL == 1:
        trajectory_remaps.append(
            ('/trajectory_ref', '/trajectory_ref_faulted')
        )
    if FAULT_CHANNEL == 2:
        trajectory_remaps.append(
            ('/torque', '/torque_raw')
        )
    if FAULT_CHANNEL == 3:
        trajectory_remaps.append(
            ('/joint_states', '/joint_states_faulted')
        )

    # ------------------------------------------------------------
    # RViz
    # ------------------------------------------------------------
    rviz_remaps = []
    if FAULT_CHANNEL == 3:
        rviz_remaps.append(
            ('/joint_states', '/joint_states_faulted')
        )

    # ------------------------------------------------------------
    # Observer remaps
    # - channel 0: observer reads tau_ext from the raw (pre-fault) topic
    # - channel 3: observer reads joint_states from the original (pre-fault) topic
    # Other channels do not affect the observer topics.
    # ------------------------------------------------------------
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

    return LaunchDescription([

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
            package='exoskeleton_utils',
            executable='external_wrench_pub',
            name='input',
            output='screen',
        ),

        Node(
            package='exoskeleton_observers',
            executable='observer_node',
            name='observer_node',
            output='screen',
            parameters=[observer_params_file],
            remappings=observer_remaps,
        ),

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
        ),
    ])