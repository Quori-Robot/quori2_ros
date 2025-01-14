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

    # get  ROS_DISTRO
    ros_distro = os.environ["ROS_DISTRO"]

    lidar1_args = {"channel_type": "udp","udp_ip":"10.2.99.111","udp_port": "50108","frame_id":"lidar1_link"}
    lidar2_args = {"channel_type": "udp","udp_ip":"10.2.99.111","udp_port": "50110","frame_id":"lidar2_link"}

    # Launch description
    lidar1 = GroupAction(
        actions=[
            # push-ros-namespace to set namespace of included nodes
            PushRosNamespace('lidar1'),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource([
                    FindPackageShare("quori_launch"),"/launch/","rplidar_a2m12_launch.py"
                ]),
                launch_arguments=lidar1_args.items(), 
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
                    FindPackageShare("quori_launch"),"/launch/","rplidar_a2m12_launch.py"
                ]), 
                launch_arguments=lidar2_args.items(),
                #launch_arguments=[{,"serial_port":"rplidar2"},parameters.items()],
            )
        ]
    )

    ld = LaunchDescription(
        [] +
        [
            lidar1,
            lidar2 
        ]
        
    )
    
    
    return ld
