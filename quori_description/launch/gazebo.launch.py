import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
            
from launch.actions import DeclareLaunchArgument, ExecuteProcess,IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration,TextSubstitution
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource



def generate_launch_description():

  use_sim_time = LaunchConfiguration('use_sim_time')
  world = LaunchConfiguration('world')

  pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')
  pkg_quori_gazebo = get_package_share_directory('quori_description')

  world_file = os.path.join(pkg_quori_gazebo , "worlds/" )
 
  bridge_params = os.path.join(get_package_share_directory('quori_description'),   'params', 'quori_bridge.yaml' )

  return LaunchDescription([

        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation (Gazebo) clock if true'),

        DeclareLaunchArgument(
            'world',
            default_value='quori.world',
            description='specify name of world file to launch'),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')),
            launch_arguments =  {'gz_args': [TextSubstitution(text='-r -z 1000000 '), world_file, world ] }.items(),
        ),

            # Spawn
        Node(package='ros_gz_sim', executable='create',
                name='model_spawner',
                arguments=[
                    '-name', 'quori',
                    '-x', '1.2',
                    '-z', '0.01',
                    '-y', '3.4',
                    '-topic', '/robot_description'],
                output='screen',
                #parameters=[{ 'use_sim_time': use_sim_time }]
        ),

        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            arguments=[
                '--ros-args',
                '-p',
                f'config_file:={bridge_params}',
            ],
            output='screen',
            #parameters=[{ 'use_sim_time': use_sim_time }]
        ),

        Node(
            package='ros_gz_image',
            executable='image_bridge',
            arguments=['camera/depth_image_raw'],
            output='screen',
            parameters=[{ 'use_sim_time': use_sim_time }]
        ),

        
        Node(
            package='ros_gz_image',
            executable='image_bridge',
            arguments=['camera/image_raw'],
            output='screen',
            parameters=[{ 'use_sim_time': use_sim_time }]
        )

  ])

