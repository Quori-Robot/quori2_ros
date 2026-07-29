from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.actions import IncludeLaunchDescription
from launch_ros.substitutions import  FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
from launch.conditions import LaunchConfigurationEquals

import os


def generate_launch_description():
    # Declare arguments
    args = [
       
        DeclareLaunchArgument('use_sim_time', default_value='false'), 
        DeclareLaunchArgument('controller_type', default_value='xbox_one'), 
        DeclareLaunchArgument('do_anim', default_value='false'), 
        DeclareLaunchArgument('anim_file', default_value=''), 
        DeclareLaunchArgument('do_head_teleop', default_value='true'), 
        
        #DeclareLaunchArgument('controller_type', default_value='xbox360'), 

        #DeclareLaunchArgument('web_address', default_value='192.168.0.28'), 
        #DeclareLaunchArgument('web_address', default_value='192.168.0.58'), 
        #DeclareLaunchArgument('web_address', default_value='192.168.0.27'), 
        #DeclareLaunchArgument('web_address', default_value='10.10.2.245'), 


    ]


    # Node configuration
    parameters = {arg.name: LaunchConfiguration(arg.name) for arg in args}

    #parameters.update({"address" : LaunchConfiguration("web_address")})

    # get  ROS_DISTRO
    ros_distro = os.environ["ROS_DISTRO"]

    # Launch description
 
    ld = LaunchDescription(
        args +
        [
        
       
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                FindPackageShare("quori_description"),"/launch/","quori_description_launch.py"
            ]), 
            launch_arguments=parameters.items(),
    
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                FindPackageShare("quori_node"),"/launch/","quori_node_launch.py"
            ]), 
            launch_arguments=parameters.items(),
    
        ),
        
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                FindPackageShare("quori_launch"),"/launch/","teleop_launch.py"
            ]), 
            launch_arguments=parameters.items(),
            condition=LaunchConfigurationEquals('do_head_teleop', 'true')
    
        ),

        Node(
            package="quori_hardware",
            executable="head_animation.py",
            remappings=[("/head_joint_vel_cmds","/joint_group_velocity_controller/commands")],
            parameters=[parameters],
            output="both",
            condition=LaunchConfigurationEquals('do_anim', 'true')
        ),
        
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                FindPackageShare("quori_hardware"),"/launch/","quori_hardware_launch.py"
            ]), 
            launch_arguments=parameters.items()
        ),
       
       
        ]
        
    )
    
    
    return ld
