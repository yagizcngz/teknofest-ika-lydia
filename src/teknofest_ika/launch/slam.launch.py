"""
slam.launch.py — LYDİA İKA SLAM Başlatıcısı
=============================================

KULLANIM — Gazebo simülasyonu:
  ros2 launch teknofest_ika slam.launch.py sim:=true

KULLANIM — Gerçek robot:
  ros2 launch teknofest_ika slam.launch.py sim:=false
  (veya sadece: ros2 launch teknofest_ika slam.launch.py)

NEDEN AYRI:
  WSL'de Gazebo world yüklenme süresi 3-6 dakika sürebilir.
  SLAM araç spawn olmadan önce başlarsa:
    - EKF'nin /odom girdisi yok → TF yayınlanamıyor
    - LiDAR taramaları TF bulamıyor → message_filter queue doluyor
    - "discarding message because the queue is full" hatası sürekli tekrar eder
  Spawn sonrası başlatılınca TF zinciri hazır, queue sorunsuz çalışır.
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('teknofest_ika')
    slam_params_file = os.path.join(
        pkg_share, 'config', 'mapper_params_online_sync.yaml')

    # Gazebo:       ros2 launch teknofest_ika slam.launch.py sim:=true
    # Gerçek robot: ros2 launch teknofest_ika slam.launch.py
    sim_arg = DeclareLaunchArgument(
        'sim',
        default_value='false',
        description='true = Gazebo simülasyonu, false = gerçek robot'
    )
    use_sim_time = LaunchConfiguration('sim')

    slam_node = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[
            slam_params_file,
            {'use_sim_time': use_sim_time}
        ]
    )

    return LaunchDescription([sim_arg, slam_node])
