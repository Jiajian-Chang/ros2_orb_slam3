#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    # Default params file from installed share
    pkg_share = get_package_share_directory('ros2_orb_slam3')
    default_params_path = os.path.join(pkg_share, 'config', 'stereo_params.yaml')
    default_settings_path = os.path.join(pkg_share, 'config', 'realsense_d455.yaml')
    default_voc_path = os.path.join(pkg_share, 'config', 'ORBvoc.txt.bin')

    left_topic_arg = DeclareLaunchArgument(
        'left_topic', default_value='/remote/d455/infra1/image_rect_raw')
    right_topic_arg = DeclareLaunchArgument(
        'right_topic', default_value='/remote/d455/infra2/image_rect_raw')
    node_name_arg = DeclareLaunchArgument(
        'node_name', default_value='stereo_slam_cpp')
    params_file_arg = DeclareLaunchArgument(
        'params_file', default_value=default_params_path)
    settings_path_arg = DeclareLaunchArgument(
        'settings_path', default_value=default_settings_path)
    voc_path_arg = DeclareLaunchArgument(
        'voc_path', default_value=default_voc_path)
    headless_arg = DeclareLaunchArgument(
        'headless', default_value='false')

    left_topic = LaunchConfiguration('left_topic')
    right_topic = LaunchConfiguration('right_topic')
    node_name = LaunchConfiguration('node_name')
    params_file = LaunchConfiguration('params_file')
    settings_path = LaunchConfiguration('settings_path')
    voc_path = LaunchConfiguration('voc_path')
    headless = LaunchConfiguration('headless')

    # Load YAML plus allow inline overrides and explicit absolute paths for settings/voc
    stereo_node = Node(
        package='ros2_orb_slam3',
        executable='stereo_node_cpp',
        name=node_name,
        output='screen',
        parameters=[params_file, {
            'left_topic': left_topic,
            'right_topic': right_topic,
            'node_name_arg': node_name,
            'settings_path': settings_path,
            'voc_path': voc_path,
            'headless': headless
        }]
    )

    return LaunchDescription([
        left_topic_arg,
        right_topic_arg,
        node_name_arg,
        params_file_arg,
        settings_path_arg,
        voc_path_arg,
        headless_arg,
        stereo_node
    ]) 