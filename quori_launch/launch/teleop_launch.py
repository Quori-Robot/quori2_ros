

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
        DeclareLaunchArgument('deadzone', default_value='0.05'), 
        DeclareLaunchArgument('autorepeaterate', default_value='10'), 
        DeclareLaunchArgument('coalesce_interval', default_value='0.001'), 
        DeclareLaunchArgument('dev_ff', default_value='/dev/input/event9'),
       
       
    ]

    # Node configuration
    parameters = {arg.name: LaunchConfiguration(arg.name) for arg in args}

    quary_launch_pkg_path = FindPackageShare("quori_launch")


    kb_teleop_config_path = [quary_launch_pkg_path,"/config/kb_teleop_config.yaml"]
    joy_teleop_config_path = [quary_launch_pkg_path,"/config/",LaunchConfiguration('controller_type'),"_config.yaml"]
    
    mux_config_path = [quary_launch_pkg_path,"/config/mux_config.yaml"]
    smoother_config_path = [quary_launch_pkg_path,"/config/smoother_config.yaml"]

    # get  ROS_DISTRO
    ros_distro = os.environ["ROS_DISTRO"]

    vs_node = LifecycleNode(
        namespace="velocity_smoother",
        package='nav2_velocity_smoother',
        name='velocity_smoother',
        executable='velocity_smoother',
        parameters=[smoother_config_path],
        remappings=[('cmd_vel','/cmd_vel_mux'),('cmd_vel_smoothed','/cmd_vel')],
        output='screen'

    )

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
        Node(
            package='teleop_twist_joy',
            name='teleop_joy',
            executable='teleop_node',
            parameters=[parameters,joy_teleop_config_path],
            output='screen',
            remappings=[('cmd_vel','cmd_vel_joy')]

        ),
        
        Node(
            package='twist_mux',
            name='twist_mux',
            executable='twist_mux',
            parameters=[mux_config_path],
            remappings=[('cmd_vel_out','cmd_vel_mux')],
            output='screen'

        ),
        vs_node,

        EmitEvent(
            event=ChangeState(
                    lifecycle_node_matcher = launch.events.process.matches_executable("velocity_smoother"),
                    transition_id = lifecycle_msgs.msg.Transition.TRANSITION_CONFIGURE,

            )
        ),
        EmitEvent(
            event=ChangeState(
                    lifecycle_node_matcher = launch.events.process.matches_executable("velocity_smoother"),
                    transition_id = lifecycle_msgs.msg.Transition.TRANSITION_ACTIVATE

            )
        )
        ]
 
    )

   
    return ld
