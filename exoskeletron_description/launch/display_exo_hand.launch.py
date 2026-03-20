from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    # Percorso al file URDF
    pkg_share = get_package_share_directory('exoskeletron_description')
    urdf_file = os.path.join(pkg_share, 'urdf', 'assembly_with_hand.urdf')

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

        #GUI per ruotare il giunto motore (pubblica su /joint_states)
        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            name='joint_state_publisher_gui',
            output='screen'
        ),

        # RViz
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', os.path.join(pkg_share, 'rviz', 'exo_display.rviz')]
        ),
    ])
