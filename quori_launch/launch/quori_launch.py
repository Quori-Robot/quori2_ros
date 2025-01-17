from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.actions import IncludeLaunchDescription
from launch_ros.substitutions import  FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os


def generate_launch_description():
    # Declare arguments
    args = [
        DeclareLaunchArgument('camera_type', default_value='gemini335'), 
        DeclareLaunchArgument('mode', default_value='navigation'),   
    ]

    # Node configuration
    parameters = {arg.name: LaunchConfiguration(arg.name) for arg in args}

    # get  ROS_DISTRO
    ros_distro = os.environ["ROS_DISTRO"]

    # Launch description
 
    ld = LaunchDescription(
        args +
        [IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                FindPackageShare("quori_launch"),"/launch/","camera_launch.py"
            ]), 
            launch_arguments=parameters.items(),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                FindPackageShare("quori_launch"),"/launch/","lidar_launch.py"
            ]), 
            launch_arguments=parameters.items(),
        ),
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
    
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                FindPackageShare("quori_hardware"),"/launch/","quori_hardware_launch.py"
            ]), 
            launch_arguments=parameters.items(),
        ),
        ]
        
    )
    
    
    return ld
