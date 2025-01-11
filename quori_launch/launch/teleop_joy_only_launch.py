

import launch
from launch.events.process import *
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.actions import EmitEvent
from launch_ros.actions import Node, LifecycleNode
from launch_ros.substitutions import  FindPackageShare
from launch_ros.events.lifecycle import ChangeState
from launch.conditions import IfCondition,LaunchConfigurationEquals

import os
import lifecycle_msgs.msg

def generate_launch_description():
    # Declare arguments
    args = [
        DeclareLaunchArgument('controller_type', default_value='xbox360'), 
        DeclareLaunchArgument('dev_name', default_value='Xbox 360 Wireless Receiver'), 
        DeclareLaunchArgument('deadzone', default_value='0.2'), 
        DeclareLaunchArgument('autorepeaterate', default_value='20'), 
        DeclareLaunchArgument('coalesce_interval', default_value='0.001'), 
        DeclareLaunchArgument('dev_ff', default_value='/dev/input/event9'),
       
       
    ]

    # Node configuration
    parameters = {arg.name: LaunchConfiguration(arg.name) for arg in args}

    quary_launch_pkg_path = FindPackageShare("quori_launch")


    kb_teleop_config_path = [quary_launch_pkg_path,"/config/kb_teleop_config.yaml"]
    joy_teleop_config_path = [quary_launch_pkg_path,"/config/",LaunchConfiguration('controller_type'),"_config.yaml"]
    
    # get  ROS_DISTRO
    ros_distro = os.environ["ROS_DISTRO"]


    # Launch description
    ld = LaunchDescription(
        args +
        [
        Node(
            package='joy_linux',
            name='joystick',
            executable='joy_linux_node',
            parameters=[parameters],
            output='screen'

        ),
        ]
 
    )

   
    return ld
