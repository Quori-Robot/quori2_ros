from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.actions import IncludeLaunchDescription
from launch_ros.substitutions import  FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition,UnlessCondition,LaunchConfigurationEquals
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

import os


def generate_launch_description():
    # Declare arguments
    args = [
        DeclareLaunchArgument('camera_type', default_value='gemini335'), 
        DeclareLaunchArgument('use_sim_time', default_value='true'), 
        DeclareLaunchArgument('mode', default_value=''),   
        DeclareLaunchArgument(name='world',  default_value='quori.world', description='specify name of world file to launch'),
        DeclareLaunchArgument(
            'map', default_value=os.path.join(get_package_share_directory("quori_nav"), 'maps', 'sim_cafe.yaml'), description='Full path to map yaml file to load'
        ),
    ]

    rviz_config_dir = os.path.join(
        get_package_share_directory('quori_launch'),
        'config',
        'rviz.rviz')

    # Node configuration
    parameters = {arg.name: LaunchConfiguration(arg.name) for arg in args}

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
            condition=IfCondition('False')
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

        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config_dir],
            output='screen',
            parameters=[{'use_sim_time': False}]
        ),
    ])

    
    
    return ld
