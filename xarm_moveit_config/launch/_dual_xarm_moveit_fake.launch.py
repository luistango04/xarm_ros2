#!/usr/bin/env python3
# Software License Agreement (BSD License)
#
# Copyright (c) 2021, UFACTORY, Inc.
# All rights reserved.
#
# Author: Vinman <vinman.wen@ufactory.cc> <vinman.cub@gmail.com>

import os
from ament_index_python import get_package_share_directory
from launch.launch_description_sources import load_python_launch_file_as_module
from launch import LaunchDescription
from launch.actions import OpaqueFunction, IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node


def launch_setup(context, *args, **kwargs):
    prefix_1 = LaunchConfiguration('prefix_1', default='L_')
    prefix_2 = LaunchConfiguration('prefix_2', default='R_')
    hw_ns = LaunchConfiguration('hw_ns', default='xarm')
    limited = LaunchConfiguration('limited', default=False)
    effort_control = LaunchConfiguration('effort_control', default=False)
    velocity_control = LaunchConfiguration('velocity_control', default=False)
    add_gripper = LaunchConfiguration('add_gripper', default=False)
    add_vacuum_gripper = LaunchConfiguration('add_vacuum_gripper', default=False)
    dof = LaunchConfiguration('dof', default=7)
    no_gui_ctrl = LaunchConfiguration('no_gui_ctrl', default=False)

    ros2_control_plugin = 'fake_components/GenericSystem'
    controllers_name = 'fake_controllers'
    moveit_controller_manager_key = 'moveit_fake_controller_manager'
    moveit_controller_manager_value = 'moveit_fake_controller_manager/MoveItFakeControllerManager'

    # robot_description
    mod = load_python_launch_file_as_module(os.path.join(get_package_share_directory('xarm_description'), 'launch', 'lib', 'xarm_description_lib.py'))
    get_dual_xarm_robot_description = getattr(mod, 'get_dual_xarm_robot_description')
    robot_description = get_dual_xarm_robot_description(
        prefix_1, prefix_2, hw_ns.perform(context).strip('/'), limited, 
        effort_control, velocity_control, 
        add_gripper, add_vacuum_gripper, 
        dof, ros2_control_plugin
    )
    # robot state publisher node
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[robot_description],
        remappings=[
            # ('joint_states', joint_states_remapping),
            ('/tf', 'tf'),
            ('/tf_static', 'tf_static'),
        ]
    )

    # xarm moveit common launch
    xarm_moveit_common_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([FindPackageShare('xarm_moveit_config'), 'launch', '_dual_xarm_moveit_common.launch.py'])),
        launch_arguments={
            'prefix_1': prefix_1,
            'prefix_2': prefix_2,
            'hw_ns': hw_ns,
            'limited': limited,
            'effort_control': effort_control,
            'velocity_control': velocity_control,
            'add_gripper': add_gripper,
            'add_vacuum_gripper': add_vacuum_gripper,
            'dof': dof,
            'no_gui_ctrl': no_gui_ctrl,
            'ros2_control_plugin': ros2_control_plugin,
            'controllers_name': controllers_name,
            'moveit_controller_manager_key': moveit_controller_manager_key,
            'moveit_controller_manager_value': moveit_controller_manager_value
        }.items(),
    )

    # joint state publisher node
    joint_state_publisher_node = Node(
        package="joint_state_publisher",
        executable="joint_state_publisher",
        name='joint_state_publisher',
        output='screen',
        parameters=[{"source_list": ["fake_controller_joint_states"]}],
    )


    return [
        robot_state_publisher_node,
        xarm_moveit_common_launch,
        joint_state_publisher_node,
    ]

def generate_launch_description():
    return LaunchDescription([
        OpaqueFunction(function=launch_setup)
    ])