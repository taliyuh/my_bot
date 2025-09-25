import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.actions import TimerAction

from launch_ros.actions import Node


def generate_launch_description():

    package_name = 'my_bot'

    # Declare world argument (only the filename!)
    declare_world = DeclareLaunchArgument(
        'world',
        default_value='empty.world',
        description='Name of the world file to load (must be in my_bot/worlds/)'
    )

    # Path to the selected world file
    world = PathJoinSubstitution([
        get_package_share_directory(package_name),
        'worlds',
        LaunchConfiguration('world')
    ])

    # Robot state publisher
    rsp = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(
                get_package_share_directory(package_name),
                'launch',
                'rsp.launch.py'
            )
        ]),
        launch_arguments={'use_sim_time': 'true'}.items()
    )

    # Gazebo with selected world
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(
                get_package_share_directory('gazebo_ros'),
                'launch',
                'gazebo.launch.py'
            )
        ]),
        launch_arguments={'world': world}.items()
    )

    # Spawner
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-topic', 'robot_description',
                   '-entity', 'my_bot'],
        output='screen'
    )

    # Spawner for joint_state_broadcaster
    spawner_joint_broad = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_broad", "--controller-manager", "/controller_manager"],
        output="screen"
    )

    # Spawner for the diff-drive controller (delay to ensure CM is up)
    spawner_diff = TimerAction(
        period=2.0,
        actions=[Node(
            package="controller_manager",
            executable="spawner",
            arguments=["diff_cont", "--controller-manager", "/controller_manager"],
            output="screen"
        )]
    )

    return LaunchDescription([
        declare_world,
        rsp,
        gazebo,
        spawn_entity,
        spawner_joint_broad,
        spawner_diff,
    ])
