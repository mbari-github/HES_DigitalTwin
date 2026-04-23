from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg_share = get_package_share_directory('exoskeleton_description')
    # Uses the exoskeleton-only URDF (no hand model)
    urdf_file = os.path.join(pkg_share, 'urdf', 'assembly.urdf')

    with open(urdf_file, 'r') as infp:
        robot_description_content = infp.read()

    return LaunchDescription([

        # Robot State Publisher — broadcasts TF from URDF
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot_description_content}]
        ),

        # GUI slider for manually rotating the motor joint (publishes on /joint_states)
        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            name='joint_state_publisher_gui',
            output='screen'
        ),

        # RViz visualization
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', os.path.join(pkg_share, 'rviz', 'exo_display.rviz')]
        ),
    ])
