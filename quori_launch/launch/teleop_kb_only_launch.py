

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
       DeclareLaunchArgument('use_sim_time', default_value='false'),
    ]

    # Node configuration
    parameters = {arg.name: LaunchConfiguration(arg.name) for arg in args}

    # Launch description
    ld = LaunchDescription(
        args +
        [
        Node(
            package='teleop_twist_keyboard',
            name='teleop_kb',
            executable='teleop_twist_keyboard',
            parameters=[parameters],
            output='screen',
            prefix = 'xterm -e',
            
            remappings=[('cmd_vel','cmd_vel_kb')]

        ),
        ]
 
    )

   
    return ld
