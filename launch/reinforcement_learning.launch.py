#
# Launch NVIDIA JetBot motor controller and camera nodes.
# This is for the original NVIDIA JetBot.
#
import os

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import ThisLaunchFileDir, LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    
    motor_controller = Node(package='jetbot_ros', node_executable='motors_waveshare',
                            output='screen', emulate_tty=True)              
    
    video_source = Node(package='ros_deep_learning', node_executable='video_source',
                        parameters=[
                            {"resource": "csi://0"},
                            {"width": 160},
                            {"height": 120},
                            {"framerate": 10.0},
                            {"flip": "rotate-180"},
                        ],
                        remappings=[
                            ("/video_source/raw", "/jetbot/camera/image_raw"),
                        ],
                        output='screen', emulate_tty=True)

    api_handler = Node(package='jetbot_ros', node_executable='api_handler',
                            output='screen', emulate_tty=True)        
                        
    return LaunchDescription([
        motor_controller,
        video_source,
        api_handler
    ])