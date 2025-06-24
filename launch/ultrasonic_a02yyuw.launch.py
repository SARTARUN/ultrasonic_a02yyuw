from launch import LaunchDescription
from launch_ros.actions import Node
import os

def generate_launch_description():
    urdf_path = os.path.join(
        os.path.dirname(__file__), '..', 'urdf', 'ultrasonic_a02yyuw_description.urdf'
    )
    urdf_path = os.path.abspath(urdf_path)

    return LaunchDescription([
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': open(urdf_path).read()}]
        ),
        Node(
            package='ultrasonic_a02yyuw',
            executable='ultrasonic_a02yyuw',
            name='ultrasonic_range_publisher',
            output='screen'
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', os.path.join(os.path.dirname(__file__), '..', 'config', 'config.rviz')],
        ),
    ])