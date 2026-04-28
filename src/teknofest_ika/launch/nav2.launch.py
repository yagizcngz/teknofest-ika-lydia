"""
nav2.launch.py — LYDİA İKA Nav2 Başlatıcısı
=============================================

BAŞLATMA SIRASI (gerçek robot):
  Terminal 1: ros2 launch teknofest_ika baslat.launch.py   (sensörler + EKF + seri_kopru)
  Terminal 2: ros2 launch teknofest_ika slam.launch.py     (harita hazır olana kadar bekle)
  Terminal 3: ros2 launch teknofest_ika nav2.launch.py     ← bu dosya
  Terminal 4: ros2 run teknofest_ika misyon_fsm            ← FSM

NOT: use_sim_time = false — gerçek robot sistem saatini kullanır.
     Gazebo simülasyonunda true yapılmalı.
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('teknofest_ika')
    nav2_params = os.path.join(pkg_share, 'config', 'nav2_params.yaml')

    # ── Launch argümanı: sim mi, gerçek robot mu? ─────────────────
    # Gazebo:      ros2 launch teknofest_ika nav2.launch.py sim:=true
    # Gerçek robot: ros2 launch teknofest_ika nav2.launch.py sim:=false
    # Varsayılan: false (gerçek robot)
    sim_arg = DeclareLaunchArgument(
        'sim',
        default_value='false',
        description='true = Gazebo simülasyonu, false = gerçek robot'
    )
    use_sim_time = LaunchConfiguration('sim')

    nav2_bringup_dir = get_package_share_directory('nav2_bringup')

    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup_dir, 'launch', 'navigation_launch.py')
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'params_file':  nav2_params,
            'use_composition': 'False',
        }.items()
    )

    return LaunchDescription([sim_arg, nav2])
