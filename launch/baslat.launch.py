import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import AppendEnvironmentVariable, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    pkg_share = get_package_share_directory('teknofest_ika')

    set_model_path = AppendEnvironmentVariable('GAZEBO_MODEL_PATH', os.path.join(pkg_share, 'models'))
    ika_ws = os.environ.get('IKA_WS', os.path.expanduser('~/ika_ws'))
    set_plugin_path = AppendEnvironmentVariable('GAZEBO_PLUGIN_PATH', os.path.join(ika_ws, 'plugins'))

    world_file = os.path.join(pkg_share, 'worlds', 'yarisma.world')

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')
        ]),
        launch_arguments={'world': world_file, 'verbose': 'true'}.items()
    )

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

    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-topic', 'robot_description', '-entity', 'teknofest_araci',
                   '-x', '0.0', '-y', '0.0', '-z', '0.1'],
        output='screen'
    )

    # ← YENİ: SLAM Toolbox node tanımı
    slam_node = Node(
        package='slam_toolbox',
        executable='sync_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'odom_frame': 'odom',
            'map_frame': 'map',
            'base_frame': 'base_link',
            'scan_topic': '/scan',
            'mode': 'mapping',
            'max_laser_range': 20.0,
            'resolution': 0.05,
            'transform_timeout': 0.5,
            'message_filter_queue_size': 20,
        }]
    )

    # ← YENİ: EKF node tanımı (odom + IMU füzyon)
    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'frequency': 30.0,
            'sensor_timeout': 0.1,
            'odom0': '/odom',
            'imu0': '/imu/data',
            'odom0_config': [
                True, True, False,   # x, y, z
                False, False, True,  # roll, pitch, yaw
                True, True, False,   # vx, vy, vz
                False, False, True,  # vroll, vpitch, vyaw
                False, False, False  # ax, ay, az
            ],
            'imu0_config': [
                False, False, False,
                True,  True,  True,
                False, False, False,
                True,  True,  True,
                True,  True,  True
            ],
        }]
    )

    return LaunchDescription([
        set_model_path,
        set_plugin_path,
        gazebo,
        robot_state_publisher,
        TimerAction(period=5.0, actions=[spawn_entity]),
        TimerAction(period=8.0, actions=[slam_node]),   # ← YENİ: araç spawn'dan 3sn sonra başlar
        TimerAction(period=12.0, actions=[ekf_node]),    # ← YENİ: aynı anda başlar
    ])
