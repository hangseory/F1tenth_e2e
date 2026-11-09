"""Frozen pbstream + session mapping/localization; no drivers or state saving."""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def launch_nodes(context):
    def value(name):
        return LaunchConfiguration(name).perform(context)

    pbstream = os.path.abspath(os.path.expanduser(value('pbstream')))
    if not os.path.isfile(pbstream):
        raise RuntimeError(f'Cartographer pbstream 없음: {pbstream}. '
                           'mapping-carto-joy / savemap-carto 로 먼저 저장하세요.')
    config = os.path.join(get_package_share_directory('race_stack'), 'config')
    basename = 'cartographer_driving.lua'
    sim = value('use_sim_time').lower() == 'true'
    return [
        Node(package='cartographer_ros', executable='cartographer_node',
             name='cartographer_node', output='screen',
             parameters=[{'use_sim_time': sim}],
             arguments=['-configuration_directory', config,
                        '-configuration_basename', basename,
                        '-load_state_filename', pbstream,
                        '-load_frozen_state=true',
                        '-start_trajectory_with_default_topics=false',
                        '-save_state_filename='],
             remappings=[('scan', value('scan_topic')), ('odom', value('odom_topic'))]),
        Node(package='cartographer_ros', executable='cartographer_occupancy_grid_node',
             name='cartographer_occupancy_grid_node', output='screen',
             parameters=[{'use_sim_time': sim, 'resolution': 0.05,
                          'publish_period_sec': 1.0}]),
        Node(package='race_stack', executable='cartographer_localizer',
             name='cartographer_localizer', output='screen',
             parameters=[{'use_sim_time': sim, 'configuration_directory': config,
                          'configuration_basename': basename,
                          'odom_topic': value('odom_topic'),
                          'init_x': float(value('init_x')),
                          'init_y': float(value('init_y')),
                          'init_yaw': float(value('init_yaw'))}]),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('pbstream'),
        DeclareLaunchArgument('scan_topic', default_value='/scan'),
        DeclareLaunchArgument('odom_topic', default_value='/odom_imu'),
        DeclareLaunchArgument('init_x', default_value='0.0'),
        DeclareLaunchArgument('init_y', default_value='0.0'),
        DeclareLaunchArgument('init_yaw', default_value='0.0'),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        OpaqueFunction(function=launch_nodes),
    ])
