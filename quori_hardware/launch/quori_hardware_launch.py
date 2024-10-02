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

def generate_launch_description():

    args = [
        DeclareLaunchArgument(name= 'use_sim_time', default_value='false',  description='Use simulation (Gazebo) clock if true')
    ]

    use_sim_time = LaunchConfiguration('use_sim_time')

        # Get URDF via xacro
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [
                    FindPackageShare("quori_description"),
                    "src","description"
                    "quori_description.urdf",
                ]
            ),
        ]
    )

    robot_description = {"robot_description": robot_description_content}

    robot_controllers_config = PathJoinSubstitution(
        [
            FindPackageShare("quori_hardware"),
            "config",
            "head_hardware_controller_config.yaml",
        ]
    )

    #pkg_share = launch_ros.substitutions.FindPackageShare(package='quori_hardware').find('quori_hardware')
    parameters = {arg.name: LaunchConfiguration(arg.name) for arg in args}

    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_controllers_config],
        remappings=[("/controller_manager/robot_description","/robot_description")],
        output="both",
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--param-file", robot_controllers_config],
        output="both",
    )

   
    robot_controller_spawner_x = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_trajectory_controller", "--param-file", robot_controllers_config],
        output="both",
    )

    robot_controller_spawner_x = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_group_position_controller", "--param-file", robot_controllers_config],
        output="both",
    )

    robot_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["pid_controller", "--param-file", robot_controllers_config],
        output="both",
    )

    head_teleop = Node(
        package="quori_hardware",
        executable="head_teleop.py",
        output="both",
    )
        # Delay start of joint_state_broadcaster after `robot_controller`
    # TODO(anyone): This is a workaround for flaky tests. Remove when fixed.
    delay_joint_state_broadcaster_after_robot_controller_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=robot_controller_spawner,
            on_exit=[joint_state_broadcaster_spawner],
        )
    )


    return launch.LaunchDescription(
        
        args + [
            control_node,
            robot_controller_spawner,
            delay_joint_state_broadcaster_after_robot_controller_spawner,
            head_teleop,
        ]
    )