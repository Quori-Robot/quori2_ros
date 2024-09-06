from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.actions import IncludeLaunchDescription
from launch_ros.substitutions import  FindPackageShare
from launch.conditions import LaunchConfigurationEquals
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os


def generate_launch_description():
    # Declare arguments
    args = [
        DeclareLaunchArgument('camera_type', default_value='astra2'),   
        DeclareLaunchArgument('enable_ir', default_value='false'),
        DeclareLaunchArgument('enable_colored_point_cloud', default_value='true'),
        DeclareLaunchArgument('pointcloud.enable', default_value='true'),
        DeclareLaunchArgument('enable_color', default_value='true')
        
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
                FindPackageShare("orbbec_camera"),"/launch/","astra2.launch.py"
            ]), 
            launch_arguments={'color_fps':'15','depth_fps':'15','color_width':'1920','color_height':'1080','depth_width':'1600','depth_height':'1200','enable_colored_point_cloud':LaunchConfiguration('enable_colored_point_cloud'),'enable_ir': LaunchConfiguration('enable_ir'),'publish_tf': 'false'}.items(), 
            condition=LaunchConfigurationEquals('camera_type', 'astra2')
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                FindPackageShare("orbbec_camera"),"/launch/","gemini_330_series.launch.py"
            ]), 
            launch_arguments={'color_fps':'15','depth_fps':'15','color_width':'1280','color_height':'720','depth_width':'1280','depth_height':'720','enable_colored_point_cloud':LaunchConfiguration('enable_colored_point_cloud'),'enable_ir': LaunchConfiguration('enable_ir'),'publish_tf': 'false'}.items(), 
            condition=LaunchConfigurationEquals('camera_type', 'gemini335')
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                FindPackageShare("orbbec_camera"),"/launch/","astra.launch.py"
            ]), 
            launch_arguments={'enable_ir': LaunchConfiguration('enable_ir')}.items(),
            condition=LaunchConfigurationEquals('camera_type', 'minipro')
        ),
        
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                FindPackageShare("realsense2_camera"),"/launch/","rs_launch.py"
            ]),    
            launch_arguments={'camera_name':'rs_camera','depth_module.profile':'1280x720x15', 'rgb_camera.profile':'1280x720x15',"pointcloud.enable":'false', 'depth_enable':'true'}.items(),
            condition=LaunchConfigurationEquals('camera_type', 'realsense')
        ),
        
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                FindPackageShare("zed_wrapper"),"/launch/","zed_camera.launch.py"
            ]), 
            launch_arguments={'enable_ir': LaunchConfiguration('enable_ir'),'camera_model': 'zed2','publish_tf':'false','publish_urdf':'false','enable_pose':'false'}.items(),
            condition=LaunchConfigurationEquals('camera_type', 'zed2')
        )
        ]
        
    )
    
    
    return ld
