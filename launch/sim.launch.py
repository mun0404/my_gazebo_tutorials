#!/usr/bin/env python3

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, ExecuteProcess, LogInfo, TimerAction, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition

def generate_launch_description():
    # Define directories for packages
    gazebo_ros_pkg = get_package_share_directory('gazebo_ros')
    turtlebot3_gazebo_launch = os.path.join(get_package_share_directory('turtlebot3_gazebo'), 'launch')

    # Define launch configurations for pose and simulation time
    simulation_time = LaunchConfiguration('use_sim_time', default='true')
    X = LaunchConfiguration('x_pose', default='0.0')
    Y = LaunchConfiguration('y_pose', default='0.0')

    # Argument for enabling and disabling rosbag recording
    ros_bag_arg = DeclareLaunchArgument(
        'record_ros_bag',
        default_value='true',
        description='Enable rosbag recording using (true/false)',
    )

    # Logger to indicate if rosbag recording is enabled
    rosbag_log = LogInfo(
        condition=IfCondition(LaunchConfiguration('record_ros_bag')),
        msg='Enabling rosbag recording'
    )

    # Define the directory for saving rosbag files
    results_dir = os.path.expanduser('/home/munawwar/gazebo_tutorials/src/walker/results')
    if not os.path.exists(results_dir):
        os.makedirs(results_dir)

    # Rosbag recording process with output directory specified
    rosbag = ExecuteProcess(
        cmd=['ros2', 'bag', 'record', '-a', '-x', '/camera/.*', '--output', results_dir],
        condition=IfCondition(LaunchConfiguration('record_ros_bag')),
        output='screen',
    )

    # Timer to stop rosbag recording after 20 seconds
    stop_recording = TimerAction(
        period=20.0,
        actions=[
            ExecuteProcess(
                cmd=['pkill', '-f', 'ros2 bag record'],
                output='screen',
            )
        ],
        condition=IfCondition(LaunchConfiguration('record_ros_bag'))
    )

    # Set environment variables
    set_turtlebot_model = SetEnvironmentVariable(name='TURTLEBOT3_MODEL', value='waffle')
    set_gazebo_model_path = SetEnvironmentVariable(
        name='GAZEBO_MODEL_PATH',
        value='$GAZEBO_MODEL_PATH:ros2 pkg prefix turtlebot3_gazebo /share/turtlebot3_gazebo/models/'
    )

    # Path to custom world
    world_path = os.path.expanduser("~/my_gazebo_tutorials/src/walker/worlds/my_custom_world.world")

    # Include Gazebo server launch description
    gazebo_server = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(gazebo_ros_pkg, 'launch', 'gzserver.launch.py')),
        launch_arguments={'world': world_path}.items()
    )

    # Include Gazebo client launch description
    gazebo_client = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(gazebo_ros_pkg, 'launch', 'gzclient.launch.py'))
    )

    # Include robot state publisher launch description
    robot_state_publisher = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(turtlebot3_gazebo_launch, 'robot_state_publisher.launch.py')),
        launch_arguments={'use_sim_time': simulation_time}.items()
    )

    # Include the spawn turtlebot launch description
    spawn_turtlebot = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(turtlebot3_gazebo_launch, 'spawn_turtlebot3.launch.py')),
        launch_arguments={'x_pose': X, 'y_pose': Y}.items()
    )

    # Create launch description
    ld = LaunchDescription([
        set_turtlebot_model,
        set_gazebo_model_path,
        ros_bag_arg,
        rosbag_log,
        rosbag,
        stop_recording,
    ])

    # Add actions to launch description
    ld.add_action(gazebo_server)
    ld.add_action(gazebo_client)
    ld.add_action(robot_state_publisher)
    ld.add_action(spawn_turtlebot)

    return ld
