#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
ROS 2 launch to automate the pipeline **from the first static transform onward**.

Includes:
  • 3 static_transform_publisher
  • octomap_server2 (with the specified parameters)
  • Nav2 bringup (navigation_launch.py)
  • Node offboard_control/obstacle_avoidance

Does NOT include the custom bridge (as requested).

Typical usage (inside the container):
  ros2 launch <your_package> drone_stack.launch.py \
      use_sim_time:=true \
      nav2_params:=/root/scripts/nav2_drone_params.yaml
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # --- Configurable arguments ---
    use_sim_time = LaunchConfiguration('use_sim_time')
    nav2_params = LaunchConfiguration('nav2_params')

    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time', default_value='true',
        description='Propagate use_sim_time where applicable')

    declare_nav2_params = DeclareLaunchArgument(
        'nav2_params', default_value='/root/scripts/nav2_drone_params.yaml',
        description='Path to the parameter file for Nav2')

    actions = [
        declare_use_sim_time,
        declare_nav2_params,
    ]

    # --- Static TFs ---
    static_default_map = Node(
        package='tf2_ros', executable='static_transform_publisher',
        name='tf_default_to_map',
        arguments=['0', '0', '0', '0', '0', '0', 'default', 'map'],
        output='screen'
    )

    static_map_odom = Node(
        package='tf2_ros', executable='static_transform_publisher',
        name='tf_map_to_odom',
        arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom'],
        output='screen'
    )

    static_depth_base = Node(
        package='tf2_ros', executable='static_transform_publisher',
        name='tf_depth_base',
        arguments=['0', '0', '0', '0', '0', '0', 'x500_depth_0/base_link', 'base_link'],
        output='screen'
    )

    actions += [static_default_map, static_map_odom, static_depth_base]

    # --- Octomap Server ---
    octomap_launch_path = os.path.join(
        get_package_share_directory('octomap_server2'), 'launch', 'octomap_server_launch.py'
    )

    octomap_include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(octomap_launch_path),
        launch_arguments={
            'input_cloud_topic': '/gazebo/points',
            'frame_id': 'map',
            'base_frame_id': 'map',
            'filter_ground': 'True',
            'ground_filter/angle': '0.2',
            'ground_filter/distance': '0.10',
            'ground_filter/plane_distance': '0.10',
            'filter_speckles': 'True',
            'height_map': 'False',
            'colored_map': 'False',
            'incremental_2D_projection': 'True',
            'use_sim_time': use_sim_time,
        }.items()
    )

    # Wait a bit for the TFs to be available
    octomap_delayed = TimerAction(period=2.0, actions=[octomap_include])
    actions.append(octomap_delayed)

    # --- Run projected_to_map after 10 seconds from octomap start ---
    projected_to_map_node = Node(
        package='offboard_control',
        executable='projected_to_map',
        name='projected_to_map',
        output='screen'
    )

    projected_to_map_delayed = TimerAction(period=12.0, actions=[projected_to_map_node])
    actions.append(projected_to_map_delayed)

    return LaunchDescription(actions)
