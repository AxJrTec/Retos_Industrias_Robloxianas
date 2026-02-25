from launch import LaunchDescription
from launch.actions import GroupAction
from launch_ros.actions import Node, PushRosNamespace

def create_group(group_name, signal_type):

    return GroupAction([
        
        PushRosNamespace(group_name),

        Node(
            name="motor_sys",
            package='motor_control',
            executable='dc_motor',
            emulate_tty=True,
            output='screen',
            parameters=[{
                'sample_time': 0.01,
                'sys_gain_K': 1.75,
                'sys_tau_T': 0.05,
                'initial_conditions': 0.0,
            }]
        ),

        Node(
            name="sp_gen",
            package='motor_control',
            executable='set_point',
            emulate_tty=True,
            output='screen',
            parameters=[{
                'amplitude': 2.0,
                'omega': 1.0,
                'signal_type': signal_type,
            }]
        ),

        Node(
            name="ctrl",
            package='motor_control',
            executable='controller',
            emulate_tty=True,
            output='screen',
            parameters=[{
                'kp': 0.576,
                'ki': 13.067,
                'kd': 0.0,
                'Ts': 0.01,
            }]
        ),
    ])

def generate_launch_description():

    group1 = create_group('group1', 'sine')
    group2 = create_group('group2', 'square')
    group3 = create_group('group3', 'triangle')

    return LaunchDescription([
        group1,
        group2,
        group3
    ])