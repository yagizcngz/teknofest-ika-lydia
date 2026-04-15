import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import AppendEnvironmentVariable, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    pkg_share = get_package_share_directory('teknofest_ika')

    set_model_path = AppendEnvironmentVariable('GAZEBO_MODEL_PATH', os.path.join(pkg_share, 'models'))
    set_plugin_path = AppendEnvironmentVariable('GAZEBO_PLUGIN_PATH', '/home/bugra16/ika_ws/plugins')

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
                   '-x', '0.0', '-y', '0.0', '-z', '0.5'],
        output='screen'
    )

    return LaunchDescription([
        set_model_path,
        set_plugin_path,
        gazebo,
        robot_state_publisher,
        TimerAction(period=5.0, actions=[spawn_entity])
    ])
