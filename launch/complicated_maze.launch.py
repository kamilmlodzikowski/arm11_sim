#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node as LaunchNode
import numpy as np

def generate_launch_description():
    os.environ['TURTLEBOT3_MODEL'] = 'waffle'
    os.environ['GAZEBO_MODEL_PATH'] = '/arm_ws/src/arm11_sim/models'
    launch_file_dir = os.path.join(get_package_share_directory('arm_11_sim'), 'launch')
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')

    x_pose = LaunchConfiguration('x_pose', default='0.0')
    y_pose = LaunchConfiguration('y_pose', default='0.0')

    goal_position_x = '9.56186' 
    goal_position_y = '6.95623'

    common_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_file_dir, 'common.launch.py')
        ),
        launch_arguments={
            'x_pose': x_pose,
            'y_pose': y_pose,
            'goal_position_x': goal_position_x,
            'goal_position_y': goal_position_y
        }.items()
    )

    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    world = os.path.join(
        get_package_share_directory('arm_11_sim'),
        'worlds',
        'complicated_maze.world'
    )

    gzserver_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gzserver.launch.py')
        ),
        launch_arguments={'world': world}.items()
    )


    ld = LaunchDescription()
    ld.add_action(common_cmd)
    ld.add_action(gzserver_cmd)
    return ld