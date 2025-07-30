from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():

    starting_number = LaunchConfiguration('starting_number')
    target_ip = LaunchConfiguration('target_IP')

    return LaunchDescription([
        DeclareLaunchArgument(
            'starting_number',
            default_value='10000',
        ),

        DeclareLaunchArgument(
            'target_IP',
            default_value='127.0.0.1',
        ),

        Node(
            package='collatz_core',
            executable='visualizer.py', 
            name='visualizer',
        ),
        
        Node(
            package='collatz_cpp_server',
            executable='cpp_server',
            name='cpp_server',
            parameters=[{'starting_number': starting_number},
                        {'target_IP': target_ip}],
        ),
    ])