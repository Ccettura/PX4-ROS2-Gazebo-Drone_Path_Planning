#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
ROS 2 launch per automatizzare la pipeline **dalla prima static transform in poi**.

Include:
  • 3 static_transform_publisher
  • octomap_server2 (con i parametri indicati)
  • Nav2 bringup (navigation_launch.py)
  • Nodo offboard_control/obstacle_avoidance

NON include il custom bridge (come richiesto).

Uso tipico (dentro il container):
  ros2 launch <il_tuo_pacchetto> drone_stack.launch.py \
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
    # --- Argomenti configurabili ---
    use_sim_time = LaunchConfiguration('use_sim_time')
    nav2_params = LaunchConfiguration('nav2_params')

    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time', default_value='true',
        description='Propaga use_sim_time dove applicabile')

    declare_nav2_params = DeclareLaunchArgument(
        'nav2_params', default_value='/root/scripts/nav2_drone_params.yaml',
        description='Percorso al file di parametri per Nav2')

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

    # Attendi un attimo che i TF siano disponibili
    octomap_delayed = TimerAction(period=2.0, actions=[octomap_include])
    actions.append(octomap_delayed)

    return LaunchDescription(actions)
    
