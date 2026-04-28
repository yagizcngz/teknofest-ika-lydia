import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import AppendEnvironmentVariable, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('teknofest_ika')

    set_model_path  = AppendEnvironmentVariable('GAZEBO_MODEL_PATH',  os.path.join(pkg_share, 'models'))
    set_plugin_path = AppendEnvironmentVariable('GAZEBO_PLUGIN_PATH', '/home/bugra16/ika_ws/plugins')

    world_file = os.path.join(pkg_share, 'worlds', 'yarisma.world')
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')
        ]),
        launch_arguments={'world': world_file, 'verbose': 'true'}.items()
    )

    # ── Robot State Publisher ─────────────────────────────────────────
    urdf_file = os.path.join(pkg_share, 'urdf', 'arac.urdf')
    with open(urdf_file, 'r') as f:
        robot_desc = f.read()
    robot_desc = robot_desc.replace("package://teknofest_ika", "file://" + pkg_share)

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_desc, 'use_sim_time': True}],
        output='screen'
    )

    # ── Gazebo Spawn ──────────────────────────────────────────────────
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-topic', 'robot_description', '-entity', 'teknofest_araci',
                   '-x', '0.0', '-y', '0.0', '-z', '0.1'],
        output='screen'
    )

    # ── EKF — robot_localization (odom + IMU füzyon) ──────────────────
    # Girdi : /odom (diff_drive) + /imu/data (MPU9250)
    # Çıktı : /odometry/filtered → Nav2 bt_navigator kullanır
    # Config: config/ekf_params.yaml
    # Kurulum: sudo apt install ros-humble-robot-localization
    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[
            os.path.join(pkg_share, 'config', 'ekf_params.yaml'),
            {'use_sim_time': True}   # yaml'daki değerleri override eder
        ]
    )

    # ── Veri Paketi — Şartname 6.14 ──────────────────────────────────
    # 3 kamera kaydı: ileri, geri, nişan kamerası
    # Kayıt ~/ika_kayitlar/<tarih>/ dizinine yazılır
    # /misyon/aktif → True gelince otomatik başlar (misyon_fsm yayınlar)
    # Manuel: ros2 topic pub /veri_paketi/kayit_baslat std_msgs/msg/Bool "data: true"
    veri_paketi_node = Node(
        package='teknofest_ika',
        executable='veri_paketi',
        name='veri_paketi',
        output='screen',
    )

    # ── SLAM Toolbox — AYRI TERMİNALDE BAŞLATILACAK ─────────────────
    # WSL'de Gazebo world yüklenme süresi 3-6 dakika sürebilir.
    # SLAM bu launch'tan çıkarıldı: araç spawn olmadan önce başlarsa
    # message_filter queue dolar ve sürekli "queue is full" hatası üretir.
    #
    # Araç spawn olduktan sonra (terminalde "Successfully spawned" görününce)
    # ayrı bir terminalde şu komutu çalıştır:
    #
    #   source ~/ika_ws/install/setup.bash
    #   ros2 launch teknofest_ika slam.launch.py
    #
    # slam.launch.py dosyası: ~/ika_ws/src/teknofest_ika/launch/slam.launch.py

    return LaunchDescription([
        set_model_path,
        set_plugin_path,
        gazebo,
        robot_state_publisher,
        # t=0s  : Gazebo başlar
        # t=5s  : Araç spawn edilir (Gazebo hazır olsun)
        # t=8s  : EKF başlar (odom + imu topic'leri hazır olsun)
        # t=8s  : Veri paketi kaydedici başlar (misyon başlayana kadar bekler)
        TimerAction(period=5.0, actions=[spawn_entity]),
        TimerAction(period=8.0, actions=[ekf_node]),
        TimerAction(period=8.0, actions=[veri_paketi_node]),
    ])
