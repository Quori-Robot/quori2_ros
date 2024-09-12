import launch
import launch_ros
import os

from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch_ros.actions import Node
from launch.substitutions import Command, LaunchConfiguration,PythonExpression,TextSubstitution
from launch.actions import IncludeLaunchDescription

def generate_launch_description():

    use_sim_time = LaunchConfiguration('use_sim_time')
    use_gui = LaunchConfiguration('use_state_pub_gui')
    camera_type = LaunchConfiguration('camera_type')

    pkg_share = launch_ros.substitutions.FindPackageShare(package='quori_description').find('quori_description')

    default_model_path = os.path.join(pkg_share, 'src/description/quori_description.urdf')

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{
            'robot_description': Command(['xacro ', LaunchConfiguration('model'), TextSubstitution(text=" use_sim:="), use_sim_time , TextSubstitution(text=" camera_type:="), camera_type ],on_stderr='warn'),
            'use_sim_time': use_sim_time
        }]
    )

    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        condition=launch.conditions.UnlessCondition(use_gui),
        parameters=[{
            'source_list': ['base_wheel_joints',"body_joint"],
            'use_sim_time': use_sim_time
        }]
    )

    joint_state_publisher_gui_node = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        condition=launch.conditions.IfCondition(use_gui),
        parameters=[{
            'use_sim_time': use_sim_time
        }]
    )

    return launch.LaunchDescription([

        DeclareLaunchArgument(name= 'use_sim_time', default_value='false',
                                            description='Use simulation (Gazebo) clock if true'),
        DeclareLaunchArgument(name='use_state_pub_gui', default_value='False',
                                            description='Flag to enable joint_state_publisher_gui'),
        DeclareLaunchArgument(name='model', default_value=default_model_path,
                                            description='Absolute path to robot urdf file'),

        DeclareLaunchArgument(name='camera_type', default_value='zed2',
                                             description='Type of depth camera'),

        joint_state_publisher_node,
        joint_state_publisher_gui_node,
        robot_state_publisher_node

    ])