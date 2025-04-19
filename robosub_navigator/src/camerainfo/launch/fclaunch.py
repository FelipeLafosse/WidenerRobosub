from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, ExecuteProcess
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        # Camera node with GStreamer pipeline
        Node(
            package='camerainfo',
            executable='frontcam',
            name='camera_node',
            parameters=[{
                'gstreamer_pipeline': 'v4l2src ! video/x-raw, width=640, height=480 ! videoconvert ! appsink',
                'frequency': 5,
                'image_topic': '/camera/image_raw'
            }]
        ),

        # Automatically launch rqt_image_view using ExecuteProcess
        ExecuteProcess(
            cmd=['ros2', 'run', 'rqt_image_view', 'rqt_image_view'],
            name='rqt_image_view',
            output='screen'
        ),
    ])
