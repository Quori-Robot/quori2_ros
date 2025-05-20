from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.actions import IncludeLaunchDescription
from launch_ros.substitutions import  FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    # Declare arguments
    args = [
        DeclareLaunchArgument('camera_type', default_value='gemini335'), 
        DeclareLaunchArgument('use_sim_time', default_value='false'), 
        DeclareLaunchArgument('mode', default_value='navigation'),   
        DeclareLaunchArgument('map', default_value=os.path.join(get_package_share_directory("quori_nav"), 'maps', 'cafe2.yaml'), description='Full path to map yaml file to load'),
        DeclareLaunchArgument('controller_type', default_value='xbox360'), 

        #DeclareLaunchArgument('web_address', default_value='192.168.0.58'), 
        DeclareLaunchArgument('web_address', default_value='192.168.0.27'), 
        #DeclareLaunchArgument('web_address', default_value='10.10.2.245'), 


    ]


    # Node configuration
    parameters = {arg.name: LaunchConfiguration(arg.name) for arg in args}

    parameters.update({"address" : LaunchConfiguration("web_address")})

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
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                FindPackageShare("quori_nav"),"/launch/","quori_nav.launch.py"
            ]), 
            launch_arguments=parameters.items(),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                FindPackageShare("quori_web_ui"),"/launch/","quori_web_ui_launch.py"
            ]), 
            launch_arguments = parameters.items(),
        ),
        ]
        
    )
    
    
    return ld
