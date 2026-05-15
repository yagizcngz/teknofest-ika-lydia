import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import AppendEnvironmentVariable, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    pkg_share = get_package_share_directory('teknofest_ika')

    ws_root = os.path.abspath(os.path.join(pkg_share, '..', '..', '..', '..'))
    set_model_path = AppendEnvironmentVariable('GAZEBO_MODEL_PATH', os.path.join(ws_root, 'models') + ':' + os.path.join(pkg_share, 'models'))
    set_plugin_path = AppendEnvironmentVariable('GAZEBO_PLUGIN_PATH', os.path.join(ws_root, 'plugins'))

    world_file = os.path.join(pkg_share, 'worlds', 'yarisma.world')

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')
        ]),
        launch_arguments={'world': world_file, 'verbose': 'false'}.items()
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

    slam_params = os.path.join(pkg_share, 'config', 'mapper_params_online_sync.yaml')

    slam_node = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[slam_params, {'use_sim_time': True}]
    )

    # ← YENİ: EKF node tanımı (odom + IMU füzyon)
    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[
            os.path.join(pkg_share, 'config', 'ekf.yaml'),
            {'use_sim_time': True}
        ]
    )

    return LaunchDescription([
        set_model_path,
        set_plugin_path,
        gazebo,
        robot_state_publisher,
        TimerAction(period=5.0, actions=[spawn_entity]),
        TimerAction(period=8.0, actions=[ekf_node]),
        TimerAction(period=12.0, actions=[slam_node]),
    ])
