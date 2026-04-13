import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, AppendEnvironmentVariable, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    pkg_name = 'teknofest_ika'
    pkg_share = get_package_share_directory(pkg_name)

    # 1. ADIM: GAZEBO'YA PARKURUN YENİ ADRESİNİ ÖĞRET
    models_dir = os.path.join(pkg_share, 'models')
    set_model_path = AppendEnvironmentVariable(
        'GAZEBO_MODEL_PATH',
        models_dir
    )

    # 1b. KAYAR ENGEL PLUGIN YOLUNU EKLE
    set_plugin_path = AppendEnvironmentVariable(
        'GAZEBO_PLUGIN_PATH',
        '/home/bugra16/ika_ws/plugins'
    )

    # 2. ADIM: PARKUR DÜNYAMIZI SEÇ
    world_file = os.path.join(pkg_share, 'worlds', 'yarisma.world')

    # 3. ADIM: GAZEBO SUNUCUSUNU BAŞLAT (gzserver)
    gzserver = ExecuteProcess(
        cmd=['gzserver', '--verbose', world_file,
             '-s', 'libgazebo_ros_init.so',
             '-s', 'libgazebo_ros_factory.so'],
        output='screen'
    )

    # 3b. GAZEBO ARAYÜZÜNÜ BAŞLAT (gzclient - EOL plugin olmadan!)
    gzclient = ExecuteProcess(
        cmd=['gzclient'],
        output='screen'
    )

    # 4. ADIM: ARABAMIZIN FİZİĞİNİ (URDF) YÜKLE
    urdf_file = os.path.join(pkg_share, 'urdf', 'arac.urdf')
    with open(urdf_file, 'r') as infp:
        robot_desc = infp.read()

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_desc, 'use_sim_time': True}]
    )

    # 5. ADIM: ARABAYI SAHNEYE İNDİR
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-topic', 'robot_description',
            '-entity', 'teknofest_araci',
            '-x', '0.0', '-y', '0.0', '-z', '0.5'
        ],
        output='screen'
    )

    # SİSTEMİ ÇALIŞTIRMA LİSTESİ
    return LaunchDescription([
        set_model_path,       # Gazebo'ya model adresini ver
        set_plugin_path,      # Kayar engel plugin yolunu ekle
        gzserver,             # Gazebo sunucusu
        gzclient,             # Gazebo arayüzü (EOL plugin yok!)
        robot_state_publisher,
        spawn_entity
    ])
