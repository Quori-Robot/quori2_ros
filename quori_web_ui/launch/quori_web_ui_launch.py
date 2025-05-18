from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration,PythonExpression,TextSubstitution
from launch_ros.actions import PushRosNamespace
from launch.actions import GroupAction,EmitEvent, RegisterEventHandler
from launch.actions import IncludeLaunchDescription
from launch_ros.descriptions import ComposableNode
from launch_ros.actions import Node
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from ament_index_python.packages import get_package_share_directory
from launch_ros.substitutions import  FindPackageShare
from launch.conditions import IfCondition,UnlessCondition,LaunchConfigurationEquals
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_xml.launch_description_sources import XMLLaunchDescriptionSource

import os


def generate_launch_description():
    # Declare arguments
    args = [
        DeclareLaunchArgument('address',   default_value='192.168.0.58'),   
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('web_port', default_value='8080'),
        DeclareLaunchArgument('video_port', default_value='8081'),
        DeclareLaunchArgument('bridge_port', default_value='8082'),
        DeclareLaunchArgument('html_folder', default_value='html')
    ]

    html_path =get_package_share_directory('quori_web_ui')

    # Node configuration
    parameters = {arg.name: LaunchConfiguration(arg.name) for arg in args}
    parameters.update({"port" : LaunchConfiguration('bridge_port')})

    qwui=Node(
            package='quori_web_ui',
            name='quori_web_ui',
            executable='quori_web_ui',
            parameters=[parameters,{"port" : LaunchConfiguration("web_port"), "html_path": [TextSubstitution(text=html_path), '/' , LaunchConfiguration('html_folder')]}],
            output='screen'
    )
    
    video_server=Node(
            package='web_video_server',
            name='web_video_server',
            executable='web_video_server',
            parameters=[parameters,{"port" : LaunchConfiguration("video_port")}],
            output='screen'
    )

    ros_bridge=IncludeLaunchDescription(
            XMLLaunchDescriptionSource([
                FindPackageShare("quori_web_ui"),"/launch/","rosbridge_websocket_launch.xml"
            ]), 
            launch_arguments=parameters.items()
    )

  
    # Launch description
    ld = LaunchDescription(
        args + [

            qwui,
            video_server,
            ros_bridge

        ]
 
    )

   
    return ld
