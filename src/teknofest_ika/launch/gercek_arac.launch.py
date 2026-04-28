"""
gercek_arac.launch.py — LYDİA İKA Gerçek Araç Launch Dosyası
─────────────────────────────────────────────────────────────────────────────
Bu dosya simülasyon OLMADAN, gerçek Jetson Orin Nano üzerinde çalışır.

Gazebo ve spawn_entity burada YOK. Gerçek sensörler (LiDAR, kamera, IMU)
ve Arduino seri köprüsü başlatılır.

Node başlatma sırası ve gerekçesi:
    t=0s  robot_state_publisher  — URDF'den TF ağacı yayınlanır
    t=1s  ekf_node               — /odom + /imu/data → /odometry/filtered
    t=1s  seri_kopru             — Arduino USB seri köprüsü (odom + IMU + lazer)
    t=3s  ackermann_converter    — /cmd_vel → /ackermann_cmd dönüşümü
                                   (Nav2 hazır olmadan önce başlaması sorun değil,
                                    gelen /cmd_vel yokken sadece watchdog çalışır)
    t=3s  veri_paketi            — Kamera kaydedici (misyon başlayana kadar bekler)
    t=5s  misyon_fsm             — Görev durum makinesi

Nav2 bu dosyadan ayrı başlatılır (nav2.launch.py) — harita yüklenip
lokalizasyon oturuncaya kadar Nav2 beklemeli.

KULLANIM:
    # Terminal 1 — temel stack
    source ~/ika_ws/install/setup.bash
    ros2 launch teknofest_ika gercek_arac.launch.py

    # Terminal 2 — Nav2 (lokalizasyon hazır olunca)
    ros2 launch teknofest_ika nav2.launch.py

    # Terminal 3 — SLAM (harita yoksa, ilk saha çıkışında)
    ros2 launch teknofest_ika slam.launch.py

─────────────────────────────────────────────────────────────────────────────
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import TimerAction
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('teknofest_ika')

    # ── Robot State Publisher ─────────────────────────────────────────────────
    # URDF'den joint ve link bilgilerini okuyarak TF ağacını yayınlar.
    # Simülasyondan farkı: use_sim_time=False (gerçek sistem saati)
    urdf_file = os.path.join(pkg_share, 'urdf', 'arac.urdf')
    with open(urdf_file, 'r') as f:
        robot_desc = f.read()
    robot_desc = robot_desc.replace("package://teknofest_ika", "file://" + pkg_share)

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_desc, 'use_sim_time': False}],
        output='screen',
    )

    # ── EKF — robot_localization ──────────────────────────────────────────────
    # Girdi : /odom (seri_kopru'dan) + /imu/data (seri_kopru'dan)
    # Çıktı : /odometry/filtered → Nav2 bt_navigator kullanır
    # TF    : odom → base_footprint
    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[
            os.path.join(pkg_share, 'config', 'ekf_params.yaml'),
            {'use_sim_time': False},
        ],
    )

    # ── Seri Köprü — Arduino USB iletişim ────────────────────────────────────
    # /odom, /imu/data yayınlar; /ackermann_cmd abone olur
    # Binary 8-byte protokol: [0xAA][CMD][D0_HI][D0_LO][D1_HI][D1_LO][CRC][0x55]
    seri_kopru_node = Node(
        package='teknofest_ika',
        executable='seri_kopru',
        name='seri_kopru',
        output='screen',
        parameters=[{'use_sim_time': False}],
    )

    # ── Ackermann Converter ───────────────────────────────────────────────────
    # Nav2 /cmd_vel (Twist) → /ackermann_cmd (AckermannDriveStamped)
    #
    # Dönüşüm formülü (bisiklet modeli):
    #   steering_angle = arctan(wheelbase × angular_z / linear_x)
    #
    # PLACEHOLDER parametreler — araç ölçülünce güncelle:
    #   wheelbase          : 0.55m (dingil arası — ölçülecek)
    #   max_steering_angle : 0.5236 rad (30°  — gerçek servo limiti ölçülecek)
    ackermann_converter_node = Node(
        package='teknofest_ika',
        executable='ackermann_converter',
        name='ackermann_converter',
        output='screen',
        parameters=[{
            'use_sim_time': False,
            'wheelbase': 0.55,           # PLACEHOLDER — ölçülecek
            'max_steering_angle': 0.5236, # PLACEHOLDER — ölçülecek
            'max_speed': 3.0,
            'cmd_vel_timeout': 0.5,
        }],
    )

    # ── Veri Paketi — Şartname 6.14 ──────────────────────────────────────────
    # /misyon/aktif → True gelince otomatik kayıt başlar
    veri_paketi_node = Node(
        package='teknofest_ika',
        executable='veri_paketi',
        name='veri_paketi',
        output='screen',
        parameters=[{'use_sim_time': False}],
    )

    # ── Misyon FSM ────────────────────────────────────────────────────────────
    # Nav2 action client, waypoint sıralaması, SHOOT, DIK_EGIM bekleme
    misyon_fsm_node = Node(
        package='teknofest_ika',
        executable='misyon_fsm',
        name='misyon_fsm',
        output='screen',
        parameters=[{'use_sim_time': False}],
    )

    return LaunchDescription([
        # t=0s: RSP — TF ağacı hemen hazır olsun
        robot_state_publisher,

        # t=1s: EKF + Seri köprü aynı anda başlar
        # Seri köprü /odom ve /imu/data yayınlamaya başlar,
        # EKF bunları alarak /odometry/filtered üretir
        TimerAction(period=1.0, actions=[ekf_node]),
        TimerAction(period=1.0, actions=[seri_kopru_node]),

        # t=3s: Dönüştürücü + veri paketi
        # EKF ve seri köprü oturmuş, /odom akıyor
        TimerAction(period=3.0, actions=[ackermann_converter_node]),
        TimerAction(period=3.0, actions=[veri_paketi_node]),

        # t=5s: Görev FSM
        # Nav2 ayrı terminalde başlatılacak, FSM onu bekler
        TimerAction(period=5.0, actions=[misyon_fsm_node]),
    ])
