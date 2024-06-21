import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():

    # ros_gz_sim_prefix = get_package_share_directory("ros_gz_sim")
    # ros_gz_sim_file= os.path.join(gazebo_prefix, 'launch', 'gz_sim.launch.py')

    # ros_gz_sim = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource(
    #         ros_gz_sim_file
    #     ),
    #     launch_arguments={'gz_args', }
    # )

    # # Include the Gazebo launch file, provided by the ros_gz_sim package
    # gazebo = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource(gazebo_file),
    #     launch_arguments={'gz_args': f'-r {world_file} -s --params-file {gazebo_params_file}'}.items(),
    # )

    # Run the spawner node from the ros_gz_sim package to spawn the robot in Ignition Gazebo
    # spawn_entity = Node(
    #     package='ros_gz_sim',
    #     executable='create',
    #     arguments=['-name', 'robot', '-topic', 'robot_description'],
    #     output='screen'
    # )

    package_name = "articubot_one"

    # publish TF of joints
    rsp_prefix = get_package_share_directory(package_name)
    rsp_file = os.path.join(rsp_prefix, "launch", "rsp.launch.py")

    # finds for gazebo_ros_pkgs and launch it
    gazebo_prefix = get_package_share_directory("gazebo_ros")
    gazebo_file = os.path.join(gazebo_prefix, "launch", "gazebo.launch.py")
   
    #params file for our gazebo simulation 
    gazebo_param_prefix = get_package_share_directory(package_name)
    gazebo_params_file = os.path.join(gazebo_param_prefix, "config", "gazebo_params.yaml")

     # Path to your custom world file
    world_prefix = get_package_share_directory(package_name)
    world_file = os.path.join(world_prefix, 'worlds', "obstacles.world")

    twist_mux_config = os.path.join(get_package_share_directory(package_name), 'config', 'twist_mux.yaml')
    
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

    # launches the controller manager for diff_drive robot from ros2_control
    diff_drive_spawner=Node(
        package="controller_manager",
        executable="spawner",
        arguments=['diff_cont']
    )

    # launches the controller manager for managing joints from ros2_control
    joint_broad_spawner=Node(
        package="controller_manager",
        executable="spawner",
        arguments=['joint_broad']
    )

    # launch teleop with remapping as teleop
    teleop = Node(
            package='teleop_twist_keyboard',
            executable='teleop_twist_keyboard',
            output='screen',
            parameters=[{
                'speed': 0.3,    # Linear speed
                'turn': 0.6,     # Angular speed
            }],
            remappings=[('/cmd_vel','/cmd_vel_teleop')],
            prefix=['xterm -e']  # Optional: open in a new terminal
        )
    
    # launch twist_mux, as a multiplexer with varying priorites, navigation has lowest priority 
    twist_mux = Node(
        package="twist_mux",
        executable="twist_mux",
        parameters=[twist_mux_config, {'use_sim_time' : True}],
        remappings=[('/cmd_vel_out', '/diff_cont/cmd_vel_unstamped')]
    )

    return LaunchDescription([
        rsp,
        gazebo,
        spawn_entity,
        diff_drive_spawner,
        joint_broad_spawner,
        teleop,
        twist_mux,
    ])