import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, EmitEvent, LogInfo,
                            RegisterEventHandler)
from launch.conditions import IfCondition
from launch.events import matches_action
from launch.substitutions import (AndSubstitution, LaunchConfiguration,
                                  NotSubstitution)
from launch_ros.actions import LifecycleNode
from launch_ros.event_handlers import OnStateTransition
from launch_ros.events.lifecycle import ChangeState
from lifecycle_msgs.msg import Transition
from launch.actions import IncludeLaunchDescription
from launch_ros.substitutions import  FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():

    # Declare arguments
    args = [

        DeclareLaunchArgument(
            'autostart', default_value='true',
            description='Automatically startup the slamtoolbox. '
                        'Ignored when use_lifecycle_manager is true.'),

        DeclareLaunchArgument(
            'use_lifecycle_manager', default_value='false',
            description='Enable bond connection during node activation'),

        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation/Gazebo clock'),

        DeclareLaunchArgument(
            'slam_params_file',
            default_value=os.path.join(get_package_share_directory("quori_nav"),
                                        'config', 'mapper_params.yaml'),
            description='Full path to the ROS2 parameters file to use for the slam_toolbox node'),

        DeclareLaunchArgument('mode', default_value='navigation'),   
    ]

    # Node configuration
    parameters = {arg.name: LaunchConfiguration(arg.name) for arg in args}

    ld = LaunchDescription(args)

    slam_ld = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            FindPackageShare("slam_toolbox"),"/launch/","online_async_launch.py"
        ]), 
        launch_arguments=parameters.items(),
    )

    ld.add_action(slam_ld)

    return ld