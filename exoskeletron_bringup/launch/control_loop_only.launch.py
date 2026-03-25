from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():

    description_pkg = get_package_share_directory('exoskeletron_description')

    urdf_file = os.path.join(description_pkg, 'urdf', 'assembly_with_hand.urdf')

    with open(urdf_file, 'r') as infp:
        robot_description_content = infp.read()

    return LaunchDescription([

        # ── Robot State Publisher ─────────────────────────────────
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot_description_content}],
        ),

        # ── Dinamica dell'esoscheletro ────────────────────────────
        Node(
            package='exoskeletron_dynamics',
            executable='exo_dynamics',
            name='dynamics',
            output='screen',
        ),

        # ── RViz ──────────────────────────────────────────────────
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', os.path.join(description_pkg, 'rviz', 'exo_display.rviz')],
        ),

        # ── Input sinusoidale (external_wrench_pub) ───────────────
        Node(
            package='exoskeletron_utils',
            executable='external_wrench_pub',
            name='input',
            output='screen',
        ),

        # ── Admittance Controller (outer loop) ────────────────────
        Node(
            package='exoskeletron_control',
            executable='admittance_controller',
            name='admittance_controller',
            output='screen',
        ),

        # ── Trajectory Controller (inner loop: PD + feedforward) ──
        Node(
            package='exoskeletron_control',
            executable='trajectory_controller',
            name='trajectory_controller',
            output='screen',
        ),
    ])