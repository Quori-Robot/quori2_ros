#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import LogInfo
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    channel_type =  LaunchConfiguration('channel_type', default='udp')
    #serial_port = LaunchConfiguration('serial_port', default='/dev/ttyUSB0')
    #serial_baudrate = LaunchConfiguration('serial_baudrate', default='256000')
    udp_ip = LaunchConfiguration('udp_ip',default='10.2.99.111')
    udp_port = LaunchConfiguration('udp_port',default='50108')

    frame_id = LaunchConfiguration('frame_id', default='laser')
    inverted = LaunchConfiguration('inverted', default='false')
    angle_compensate = LaunchConfiguration('angle_compensate', default='true')
    scan_mode = LaunchConfiguration('scan_mode', default='Sensitivity')
    
    return LaunchDescription([

        DeclareLaunchArgument(
            'channel_type',
            default_value=channel_type,
            description='Specifying channel type of lidar'),
        
        DeclareLaunchArgument(
            'udp_ip',
            default_value=udp_ip,
            description='Specifying udp ip address to connected lidar'),

        DeclareLaunchArgument(
            'udp_port',
            default_value=udp_port,
            description='Specifying udp port to connected lidar'),

        #DeclareLaunchArgument(
        #    'serial_port',
        #    default_value=serial_port,
        #    description='Specifying usb port to connected lidar'),

        #DeclareLaunchArgument(
        #    'serial_baudrate',
        #    default_value=serial_baudrate,
        #    description='Specifying usb port baudrate to connected lidar'),
        
        DeclareLaunchArgument(
            'frame_id',
            default_value=frame_id,
            description='Specifying frame_id of lidar'),

        DeclareLaunchArgument(
            'inverted',
            default_value=inverted,
            description='Specifying whether or not to invert scan data'),

        DeclareLaunchArgument(
            'angle_compensate',
            default_value=angle_compensate,
            description='Specifying whether or not to enable angle_compensate of scan data'),
        DeclareLaunchArgument(
            'scan_mode',
            default_value=scan_mode,
            description='Specifying scan mode of lidar'),


        Node(
            package='rplidar_ros',
            executable='rplidar_node',
            name='rplidar_node',
            parameters=[{'channel_type':channel_type,
                         'udp_port': udp_port,
                         'udp_ip': udp_ip,
                         'frame_id': frame_id,
                         'inverted': inverted,
                         'angle_compensate': angle_compensate}],
            output='screen'),
    ])

