from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([

        Node(
            package='signal_processing',
            executable='signal_gen',
            name='signal_gen',
            output='screen'
        ),

        Node(
            package='signal_processing',
            executable='signal_proc',
            name='signal_proc',
            output='screen'
        ),

        Node(
            package='rqt_plot',
            executable='rqt_plot',
            name='rqt_plot',
            output='screen',
            arguments=['/proc_signal/data', '/signal/data']
        ) 
    ])