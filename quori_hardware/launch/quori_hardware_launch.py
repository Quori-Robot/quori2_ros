import launch
import launch_ros
import os

from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler
from launch_ros.actions import Node
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution,FindExecutable
from launch.actions import IncludeLaunchDescription
from launch_ros.substitutions import FindPackageShare
from launch.event_handlers import OnProcessExit
from launch.conditions import IfCondition,UnlessCondition

def generate_launch_description():

    args = [
        DeclareLaunchArgument(name= 'use_sim_time', default_value='false',  description='Use simulation (Gazebo) clock if true'),
        DeclareLaunchArgument('controller_type', default_value='xbox_one'), 
    ]

    use_sim_time = LaunchConfiguration('use_sim_time')

    robot_controllers_config = PathJoinSubstitution(
        [
            FindPackageShare("quori_hardware"),
            "config",
            "head_hardware_controller_config.yaml",
        ]
    )

    joy_teleop_config_path = [ FindPackageShare("quori_launch"),"/config/",LaunchConfiguration('controller_type'),"_config.yaml"]
 

    #pkg_share = launch_ros.substitutions.FindPackageShare(package='quori_hardware').find('quori_hardware')
    parameters = {arg.name: LaunchConfiguration(arg.name) for arg in args}

    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_controllers_config],
        remappings=[("/controller_manager/robot_description","/robot_description")],
        output="both",
        condition=UnlessCondition(use_sim_time),
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["head_joint_state_broadcaster", "--param-file", robot_controllers_config],
        output="both",
        condition=UnlessCondition(use_sim_time),
    )

    joint_group_pos_ctrl_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_group_position_controller", "--param-file", robot_controllers_config],
        output="both",
        condition=UnlessCondition(use_sim_time),
    )

    neck_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["neck_pos_ctrl", "--param-file", robot_controllers_config],
        output="both",
        condition=UnlessCondition(use_sim_time),
    )

    pan_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["pan_pos_ctrl", "--param-file", robot_controllers_config],
        output="both",
        condition=UnlessCondition(use_sim_time),
    )
    
    tilt_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["tilt_pos_ctrl", "--param-file", robot_controllers_config],
        output="both",
        condition=UnlessCondition(use_sim_time),
    )

    head_teleop = Node(
        package="quori_hardware",
        executable="head_teleop.py",
        remappings=[("/head_joint_pos_cmds","/joint_group_position_controller/commands")],
        output="both",
        parameters=[parameters,joy_teleop_config_path],
    )
        # Delay start of joint_state_broadcaster after `robot_controller`
    # TODO(anyone): This is a workaround for flaky tests. Remove when fixed.
    delay_joint_state_broadcaster_after_robot_controller_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_group_pos_ctrl_spawner,
            on_exit=[joint_state_broadcaster_spawner],
        ),
        condition=UnlessCondition(use_sim_time),
    )


    return LaunchDescription(
        
        args + [
            control_node,
            #pan_controller_spawner,
            #tilt_controller_spawner,
            #neck_controller_spawner,
            joint_group_pos_ctrl_spawner,
            delay_joint_state_broadcaster_after_robot_controller_spawner,
            head_teleop,
        ]
    )