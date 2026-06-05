#!/usr/bin/env python3
"""
gazebo.launch.py
================
Gazebo simülasyonunu başlatır (araba modeli dsr_gazebo içinde spawn edilir).

Kullanım:
  ros2 launch end_effector_ros2 gazebo.launch.py
  ros2 launch end_effector_ros2 gazebo.launch.py model:=h2515 color:=white

Parametreler:
  model       (string, default: h2515)  — robot modeli
  color       (string, default: white)  — robot rengi

Sıra:
  Terminal 1: ros2 launch end_effector_ros2 gazebo.launch.py
  Terminal 2: ros2 launch end_effector_ros2 end_effector.launch.py simulation:=true use_gazebo:=true
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, IncludeLaunchDescription,
    TimerAction, LogInfo
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    dsr_gazebo = get_package_share_directory('dsr_gazebo2')

    # ── Argümanlar ────────────────────────────────────────────────────────
    args = [
        DeclareLaunchArgument('model', default_value='h2515',
            description='Doosan robot modeli'),
        DeclareLaunchArgument('color', default_value='white',
            description='Robot rengi'),
    ]

    # ── Gazebo + DSR robot ────────────────────────────────────────────────
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(dsr_gazebo, 'launch', 'dsr_gazebo.launch.py')
        ),
        launch_arguments={
            'model': LaunchConfiguration('model'),
            'color': LaunchConfiguration('color'),
        }.items(),
    )

    # ── Zımpara velocity controller — 6sn sonra otomatik yükle ───────────
    zimpara_spawner = TimerAction(
        period=6.0,
        actions=[
            Node(
                package='controller_manager',
                executable='spawner',
                name='zimpara_spawner',
                output='screen',
                arguments=[
                    'zimpara_velocity_controller',
                    '--controller-manager', '/gz/controller_manager',
                ],
            )
        ]
    )
        # joint_state_broadcaster — 6sn sonra yükle
    joint_state_broadcaster = TimerAction(
        period=6.0,
        actions=[
            Node(
                package='controller_manager',
                executable='spawner',
                name='joint_state_broadcaster_spawner',
                output='screen',
                arguments=[
                    'joint_state_broadcaster',
                    '--controller-manager', '/gz/controller_manager',
                ],
            )
        ]
    )

    log = LogInfo(msg=[
        '\n',
        '╔══════════════════════════════════════════╗\n',
        '║        Gazebo Simülasyonu Başlatıldı     ║\n',
        '╚══════════════════════════════════════════╝\n',
        '  Robot model : ', LaunchConfiguration('model'), '\n',
        '  Robot color : ', LaunchConfiguration('color'), '\n',
        '\n',
        '  Sonraki adım:\n',
        '  ros2 launch end_effector_ros2 end_effector.launch.py \\\n',
        '    simulation:=true use_gazebo:=true\n',
    ])

    return LaunchDescription([
        *args,
        log,
        gazebo_launch,
        zimpara_spawner,
        joint_state_broadcaster,
    ])