from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    # Percorso al file URDF
    description_pkg = get_package_share_directory('exoskeletron_description')
    launch_pkg      = get_package_share_directory('exoskeletron_bringup')


    urdf_file = os.path.join(description_pkg, 'urdf', 'assembly_with_hand.urdf')
    observer_params_file = os.path.join(launch_pkg, 'config', 'observer_params.yaml')


    # Legge il contenuto del file URDF
    with open(urdf_file, 'r') as infp:
        robot_description_content = infp.read()

    return LaunchDescription([

        # Robot State Publisher
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot_description_content}]
        ),

        Node(
            package='exoskeletron_dynamics',
            executable='exo_dynamics',
            name='dynamics',
            output='screen'
        ),

        # RViz
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', os.path.join(description_pkg, 'rviz', 'exo_display.rviz')]
        ),

        # Input su /exo_dynamics/external_wrench
        Node(
            package='exoskeletron_utils',
            executable='external_wrench_pub',
            name='input',
            output='screen',
        ),

        # LOGGER
        # Node(
        #     package='exoskeletron_utils',
        #     executable='logger',
        #     name='logger',
        #     output='screen',
        # ),


        # Nodes for control loop - DA RIVEDERE
        # Admittance Controller
        Node(
            package='exoskeletron_control',
            executable='admittance_controller',
            name='admittance_controller',
            output='screen',
        ),
        
        # Trajectory Controller (PD + compensazione feedforward gravità e inerzia)
        Node(
            package='exoskeletron_control',
            executable='trajectory_controller',
            name='trajectory_controller',
            output='screen',
        ),

        Node(
            package='exoskeletron_dynamics',
            executable='ekf_observer',
            name='ekf_observer',
            output='screen',
            parameters=[observer_params_file]
        ),

    ])
