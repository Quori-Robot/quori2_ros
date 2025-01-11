from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import PushRosNamespace
from launch.actions import GroupAction
from launch.actions import IncludeLaunchDescription
from launch_ros.substitutions import  FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os



def generate_launch_description():
    # Declare arguments
    args = [
        #DeclareLaunchArgument('camera_type', default_value='astra2'),     
    ]

 # Node configuration
    parameters = {arg.name: LaunchConfiguration(arg.name) for arg in args}

    # get  ROS_DISTRO
    ros_distro = os.environ["ROS_DISTRO"]

    # Launch description
    lidar1 = GroupAction(
        actions=[
            # push-ros-namespace to set namespace of included nodes
            PushRosNamespace('lidar1'),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource([
                    FindPackageShare("rplidar_ros"),"/launch/","rplidar_a2m12_launch.py"
                ]),
                launch_arguments={"frame_id":"lidar1_link","serial_port":"/dev/rplidar1"}.items(), 
                #launch_arguments=[{"frame_id":"lidar1_base_link","serial_port":"rplidar1"},parameters.items()],
            )
        ]
    )
    lidar2 = GroupAction(
        actions=[
            # push-ros-namespace to set namespace of included nodes
            PushRosNamespace('lidar2'),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource([
                    FindPackageShare("rplidar_ros"),"/launch/","rplidar_a2m12_launch.py"
                ]), 
                launch_arguments={"frame_id":"lidar2_link","serial_port":"/dev/rplidar2"}.items(),
                #launch_arguments=[{,"serial_port":"rplidar2"},parameters.items()],
            )
        ]
    )

    ld = LaunchDescription(
        args +
        [
            lidar1,
            lidar2 
        ]
        
    )
    
    
    return ld
