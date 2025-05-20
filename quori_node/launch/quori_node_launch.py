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
import os


def generate_launch_description():
    # Declare arguments
    args = [
        DeclareLaunchArgument('ethernet_interface', default_value='eth0'),   
        DeclareLaunchArgument('robot_ip_address', default_value='10.2.99.111'),   
        DeclareLaunchArgument('wheelbase', default_value='0.3225'), 
        DeclareLaunchArgument('wheelradius', default_value='0.065'), 
        DeclareLaunchArgument('xy_offset', default_value='0.117935'), 
        DeclareLaunchArgument('publish_odom_tf', default_value='True'),
        DeclareLaunchArgument('publish_wheel_joint_states', default_value='True'),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('swap_left_right', default_value='False'),
        DeclareLaunchArgument('mtr1_dir', default_value='1'),
        DeclareLaunchArgument('mtr2_dir', default_value='1'),
        DeclareLaunchArgument('mtr3_dir', default_value='1'),
    ]

    # Node configuration
    parameters = {arg.name: LaunchConfiguration(arg.name) for arg in args}
    #parameters_motors_real = {arg.name: LaunchConfiguration(arg.name) for arg in args_motors_real}
    #parameters_motors_sim = {arg.name: LaunchConfiguration(arg.name) for arg in args_motors_sim}

    swap_left_right = PythonExpression(["'True' if '", LaunchConfiguration("use_sim_time"), "' == 'true' else '",  LaunchConfiguration('swap_left_right'), "'"])
    mtr1_dir = PythonExpression(["-1 if '", LaunchConfiguration("use_sim_time"), "' == 'true' else ", LaunchConfiguration('mtr1_dir')])
    mtr2_dir = PythonExpression(["-1 if '", LaunchConfiguration("use_sim_time"), "' == 'true' else ", LaunchConfiguration('mtr2_dir')])

    motor_parameters =  {'swap_left_right': swap_left_right, 'mtr1_dir' : mtr1_dir, 'mtr2_dir' : mtr2_dir }

    # get  ROS_DISTRO
    ros_distro = os.environ["ROS_DISTRO"]

    qnode=Node(
            package='quori_node',
            name='quori_node',
            executable='quori_node',
            parameters=[parameters, motor_parameters],
            output='screen'
    )
    
    qcmd=Node(
        package='quori_node',
        name='quori_cmd',
        executable='quori_cmd',
        parameters=[parameters,motor_parameters],
        output='screen',
    )

    qivt=Node(
        package='quori_node',
        name='quori_ivt',
        executable='quori_ivt',
        parameters=[parameters,motor_parameters],
        output='screen',
        condition=UnlessCondition(LaunchConfiguration('use_sim_time'))
    )
  
    # Launch description
    ld = LaunchDescription(
        args + [

            qnode,
            qcmd,
            qivt,

            RegisterEventHandler(
                OnProcessExit(
                    target_action=qcmd,
                    on_exit=[EmitEvent(event=Shutdown(reason="System Shut Down"))]
                )
            )


        ]
 
    )

   
    return ld
