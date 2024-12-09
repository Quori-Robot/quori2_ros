from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration,PythonExpression,TextSubstitution
from launch_ros.actions import PushRosNamespace
from launch.actions import GroupAction
from launch.actions import IncludeLaunchDescription
from launch_ros.descriptions import ComposableNode
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch_ros.substitutions import  FindPackageShare
from launch.conditions import IfCondition,LaunchConfigurationEquals
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os


def generate_launch_description():
    # Declare arguments
    args = [
        DeclareLaunchArgument('ethernet_interface', default_value='eth0'),   
        DeclareLaunchArgument('robot_ip_address', default_value='10.2.99.111'),   
        DeclareLaunchArgument('wheelbase', default_value='0.324'), 
        DeclareLaunchArgument('wheelradius', default_value='0.065'), 
        DeclareLaunchArgument('xy_offset', default_value='0.153'), 
        DeclareLaunchArgument('publish_odom_tf', default_value='True'),
        DeclareLaunchArgument('publish_wheel_joint_states', default_value='True'),
        DeclareLaunchArgument('swap_left_right', default_value='False'),
        DeclareLaunchArgument('mtr1_dir', default_value='-1'),
        DeclareLaunchArgument('mtr2_dir', default_value='-1'),
        DeclareLaunchArgument('mtr3_dir', default_value='1')
    ]

    # Node configuration
    parameters = {arg.name: LaunchConfiguration(arg.name) for arg in args}

    # get  ROS_DISTRO
    ros_distro = os.environ["ROS_DISTRO"]

  
    # Launch description
    ld = LaunchDescription(
        args +
        [Node(
            package='quori_node',
            name='quori_node',
            executable='quori_node',
            parameters=[parameters],
            output='screen',
            remappings=[("quori_joint_states","joint_states")]

        ),
        
        Node(
            package='quori_node',
            name='quori_cmd',
            executable='quori_cmd',
            parameters=[parameters],
            output='screen',
        )
        ]
 
    )

   
    return ld
