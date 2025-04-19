import subprocess
import launch
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([

        # Open a new terminal and run the second node
        ExecuteProcess(
            cmd=['gnome-terminal', '--', 'bash', '-c', 'ros2 run motorinfo motorcommand'],
            output='screen',
        ),

        Node(
            package='sensorinfo',
            namespace='sensor',
            executable='depthIMU',
            name='DepthIMU'
        ),
        Node(
            package='sensorinfo',
            namespace='sensor',
            executable='hydrophone',
            name='hydrophone'
        ),
        Node(
            package='motorinfo',
            namespace='drive',
            executable='motorauv',
            name='propellers'
        ),
    ])