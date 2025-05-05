#! /usr/bin/env python3

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, TextSubstitution
from launch_ros.actions import Node, SetParameter
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_xml.launch_description_sources import XMLLaunchDescriptionSource
import os

camera_host_arg = DeclareLaunchArgument('camera_hostname', default_value='192.168.0.9')
lidar_host_arg = DeclareLaunchArgument('lidar_hostname', default_value='192.168.0.10')
imu_serial_arg = DeclareLaunchArgument('imu_port', default_value='/dev/ttyUSB0')

# not needed
# gps_serial_arg = DeclareLaunchArgument('gps_port', default_value='/dev/ttyACM0')

def routecam():
    return Node(
        package='routecam_ros2',
        executable='routecam',
        name='routecam',
        parameters=[
            {"hostname": LaunchConfiguration("camera_hostname")}
        ],
        respawn=True,
    )
    
def imu():
    return Node(
        package='imu_driver',
        executable='imu_driver_exe',
        name='imu_driver_exe',
        parameters=[
            {"port_name": LaunchConfiguration("imu_port")}
        ],
        respawn=True,
    )
    
def gps():
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('ublox_gps'), "launch/ublox_gps_node-launch.py")
        ),
    )

def lidar():
    return IncludeLaunchDescription(
        XMLLaunchDescriptionSource(
            os.path.join(get_package_share_directory('ouster_ros'), "launch/sensor.launch.xml")
        ),
        launch_arguments = {
            'sensor_hostname' : LaunchConfiguration("lidar_hostname"),
            'attempt_reconnect' : "true",
            'viz': "false",
        }.items(),
    )

def generate_launch_description():
    
    return LaunchDescription([
        camera_host_arg,
        lidar_host_arg, 
        imu_serial_arg, 
        routecam(),
        imu(),
        gps(),
        lidar(),
    ])
    