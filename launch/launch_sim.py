import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    package_name = "articubot_one"

    # Paths to the launch files
    rsp_prefix = get_package_share_directory(package_name)
    rsp_file = os.path.join(rsp_prefix, "launch", "rsp.launch.py")

    gazebo_prefix = get_package_share_directory("gazebo_ros")
    gazebo_file = os.path.join(gazebo_prefix, "launch", "gazebo.launch.py")
 
    gazebo_param_prefix = get_package_share_directory(package_name)
    gazebo_params_file = os.path.join(gazebo_param_prefix, "config", "gazebo_params.yaml")

     # Path to your custom world file
    world_prefix = get_package_share_directory(package_name)
    world_file = os.path.join(world_prefix, 'worlds', "obstacles.world")

    teleop = Node(
            package='teleop_twist_keyboard',
            executable='teleop_twist_keyboard',
            name='teleop_twist_keyboard',
            output='screen',
            parameters=[{
                'speed': 0.3,    # Linear speed
                'turn': 0.6,     # Angular speed
            }],
            remappings=[('/cmd_vel','/diff_cont/cmd_vel_unstamped')],
            prefix=['xterm -e']  # Optional: open in a new terminal
        )
    
    # Include the robot description launch file
    rsp = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(rsp_file),
        launch_arguments={'use_sim_time': 'true', 'use_ros2_control': 'true'}.items()
    )

    # Include the Gazebo launch file, provided by the gazebo_ros package
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(gazebo_file),
        launch_arguments={'world': world_file, 'extra_gazebo_args': 'ros-args --params-file' + gazebo_params_file}.items(),
    )

    # Run the spawner node from the gazebo_ros package. The entity name doesn't really matter if you only have a single robot.
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-topic', 'robot_description', '-entity', 'robot'],
        output='screen'
    )

    diff_drive_spawner=Node(
        package="controller_manager",
        executable="spawner",
        arguments=['diff_cont']
    )

    joint_broad_spawner=Node(
        package="controller_manager",
        executable="spawner",
        arguments=['joint_broad']
    )

    return LaunchDescription([
        rsp,
        gazebo,
        spawn_entity,
        diff_drive_spawner,
        joint_broad_spawner,
        teleop
    ])