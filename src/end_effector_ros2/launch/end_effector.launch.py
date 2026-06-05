#!/usr/bin/env python3
"""
end_effector.launch.py
======================
Tüm End Effector ROS2 düğümlerini başlatır (Gazebo hariç).

Kullanım:
  ros2 launch end_effector_ros2 end_effector.launch.py
  ros2 launch end_effector_ros2 end_effector.launch.py simulation:=true use_gazebo:=true
Parametreler:
  simulation        (bool,   default: false) — CAN simülasyon modu
  use_gazebo_cam    (bool,   default: false) — Gazebo kamerasını kullan
  use_gazebo        (bool,   default: false) — gazebo_bridge başlat (IK dahil)
  can_port          (string, default: /dev/ttyUSB0)
  baudrate          (int,    default: 2000000)
  model_name        (string, default: YOLO26s.pt)
  camera_index      (int,    default: 0)
  stream_fps        (int,    default: 30)
  doosan_ip         (string, default: 192.168.137.100)
  doosan_port       (int,    default: 12345)

Sıra:
  Terminal 1: ros2 launch end_effector_ros2 gazebo.launch.py spawn_car:=true
  Terminal 2: ros2 launch end_effector_ros2 end_effector.launch.py simulation:=true use_gazebo:=true
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, RegisterEventHandler, Shutdown
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    # ── Argümanlar ────────────────────────────────────────────────────────
    args = [
        DeclareLaunchArgument('simulation',       default_value='false',
            description='CAN Bus simülasyon modu'),
        DeclareLaunchArgument('use_gazebo_cam',   default_value='false',
            description='Gazebo kamerasını kullan'),
        DeclareLaunchArgument('use_gazebo',       default_value='false',
            description='gazebo_bridge düğümünü başlat'),
        DeclareLaunchArgument('can_port',         default_value='/dev/ttyUSB0',
            description='CAN Bus seri port'),
        DeclareLaunchArgument('baudrate',         default_value='2000000',
            description='CAN baudrate'),
        DeclareLaunchArgument('model_name',       default_value='YOLO26s.pt',
            description='YOLO model dosya adı'),
        DeclareLaunchArgument('camera_index',     default_value='0',
            description='USB kamera indeksi'),
        DeclareLaunchArgument('stream_fps',       default_value='30',
            description='Kamera FPS'),
        DeclareLaunchArgument('doosan_ip',        default_value='192.168.137.100',
            description='Doosan controller IP'),
        DeclareLaunchArgument('doosan_port',      default_value='12345',
            description='Doosan DRFL port'),
    ]

    # ── vision_node ───────────────────────────────────────────────────────
    vision = Node(
        package='end_effector_ros2',
        executable='vision_node',
        name='vision_node',
        output='screen',
        parameters=[{
            'model_name':     LaunchConfiguration('model_name'),
            'camera_index':   LaunchConfiguration('camera_index'),
            'stream_fps':     LaunchConfiguration('stream_fps'),
            'use_gazebo_cam': LaunchConfiguration('use_gazebo_cam'),
        }],
    )

    # ── can_node ──────────────────────────────────────────────────────────
    can = Node(
        package='end_effector_ros2',
        executable='can_node',
        name='can_node',
        output='screen',
        parameters=[{
            'port':         LaunchConfiguration('can_port'),
            'baudrate':     LaunchConfiguration('baudrate'),
            'simulation':   LaunchConfiguration('simulation'),
            'use_drfl':     True,
            'use_soem':     False,
            'doosan_ip':    LaunchConfiguration('doosan_ip'),
            'doosan_port':  LaunchConfiguration('doosan_port'),
            'publish_rate': 10.0,
        }],
    )

    # ── logic_node ────────────────────────────────────────────────────────
    logic = Node(
        package='end_effector_ros2',
        executable='logic_node',
        name='logic_node',
        output='screen',
        parameters=[{
            'simulation': LaunchConfiguration('simulation'),
        }],
    )

    # ── gui_node ──────────────────────────────────────────────────────────
    gui = Node(
        package='end_effector_ros2',
        executable='gui_node',
        name='gui_node',
        output='screen',
    )

    # ── gazebo_bridge — use_gazebo:=true ise ─────────────────────────────
    gazebo_bridge = Node(
        package='end_effector_ros2',
        executable='gazebo_bridge',
        name='gazebo_bridge',
        output='screen',
        condition=IfCondition(LaunchConfiguration('use_gazebo')),
    )

    log = LogInfo(msg=[
        '\n',
        '╔══════════════════════════════════════════╗\n',
        '║   End Effector ROS2 Sistemi Başlatıldı   ║\n',
        '╚══════════════════════════════════════════╝\n',
        '  simulation      : ', LaunchConfiguration('simulation'),       '\n',
        '  use_gazebo      : ', LaunchConfiguration('use_gazebo'),       '\n',
        '  use_gazebo_cam  : ', LaunchConfiguration('use_gazebo_cam'),   '\n',
        '  can_port        : ', LaunchConfiguration('can_port'),         '\n',
        '  model_name      : ', LaunchConfiguration('model_name'),       '\n',
        '  doosan_ip       : ', LaunchConfiguration('doosan_ip'),        '\n',
    ])

    # GUI kapandığında tüm launch sistemi kapat
    on_gui_exit = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=gui,
            on_exit=[Shutdown()],
        )
    )

    return LaunchDescription([
        *args,
        log,
        vision,
        can,
        logic,
        gui,
        gazebo_bridge,
        on_gui_exit,
    ])