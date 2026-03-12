from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='motor_input',
            executable='set_point',
            name='set_point',
            output='screen',
            emulate_tty=True,
            parameters=[
                {'signal_type': 'sine'},
                {'amplitude': 10.0},
                {'omega': 2.0}
            ]
        )
    ])