#!/usr/bin/env python3
#
# Licensed under the Apache License, Version 2.0 (the "License");
# …
# Authors: Arshad Mehmood

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition

from webots_ros2_driver.webots_launcher import WebotsLauncher
from launch_ros.actions import Node

from launch import actions, events
from launch.event_handlers import OnProcessExit

def fix_mesh_path_error(package_dir):
    wbt_path = os.path.join(package_dir,'worlds','couliglig_multiple_bot.wbt')
    with open(wbt_path, "r") as file:
        content = file.read()

    newContent = content.replace("..",package_dir + '/stl_filessss')

    # rewrite the wbt content
    with open(wbt_path, "w") as file:
        file.write(newContent)


def generate_launch_description():
    ld = LaunchDescription()

    # ─── Declare common arguments ───
    use_sim_time     = LaunchConfiguration('use_sim_time',     default='true')
    enable_drive     = LaunchConfiguration('enable_drive',     default='false')
    enable_rviz      = LaunchConfiguration('enable_rviz',      default='true')
    rviz_config_file = LaunchConfiguration('rviz_config_file', default=os.path.join(
                          get_package_share_directory('turtlebot3_multi_robot'),
                          'rviz', 'multi_nav2_default_view.rviz'))
    nav_params_file  = LaunchConfiguration('nav_params_file',  default=os.path.join(
                          get_package_share_directory('turtlebot3_multi_robot'),
                          'params', 'nav2_params.yaml'))

    ld.add_action(DeclareLaunchArgument('use_sim_time',     default_value=use_sim_time,     description='Use simulator time'))
    ld.add_action(DeclareLaunchArgument('enable_drive',     default_value=enable_drive,     description='Enable robot-drive node'))
    ld.add_action(DeclareLaunchArgument('enable_rviz',      default_value=enable_rviz,      description='Enable RViz'))
    ld.add_action(DeclareLaunchArgument('rviz_config_file', default_value=rviz_config_file, description='Path to RViz config'))
    ld.add_action(DeclareLaunchArgument('nav_params_file',  default_value=nav_params_file,  description='Path to Nav2 params'))

    # ─── Launch Webots itself ───
    couliglig_dir = get_package_share_directory('couliglig_bot')
    fix_mesh_path_error(couliglig_dir)
    webots = WebotsLauncher(
        world=os.path.join(couliglig_dir, 'worlds', 'couliglig_multiple_bot.wbt'),
        mode='realtime',
        ros2_supervisor=True
    )
    ld.add_action(webots)
    ld.add_action(webots._supervisor)

    # ─── Static Map Server + Lifecycle Manager ───
    remappings = [('/tf', 'tf'), ('/tf_static', 'tf_static')]
    ld.add_action(Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[{'yaml_filename': os.path.join(couliglig_dir, 'maps', 'my_map2.yaml')}],
        remappings=remappings
    ))
    ld.add_action(Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_map_server',
        output='screen',
        parameters=[
            {'use_sim_time': use_sim_time},
            {'autostart': True},
            {'node_names': ['map_server']}
        ]
    ))

    # ─── Per-robot bring-up ───
    robots = [
        {'name': 'robot1', 'x': '0.825591', 'y': '0.298411', 'z': '0.0863088'},
        {'name': 'robot2', 'x': '4.8182', 'y': '1.09087', 'z': '0.0863088'},
    ]
    nav_launch_dir = os.path.join(
        get_package_share_directory('turtlebot3_multi_robot'),
        'launch', 'nav2_bringup'
    )

    for r in robots:
        namespace = r['name']

        # 1) start your simple_robot_launch (which fires WebotsController, robot_state_publisher, etc.)
        ld.add_action(IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(
                    couliglig_dir,
                    'launch', 'robot_launch_multiple.py'
                )
            ),
            launch_arguments=[
                ('namespace', namespace),
            ]
        ))

        # 2) bring up Nav2 under that same namespace
        ld.add_action(IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(nav_launch_dir, 'bringup_launch.py')),
            launch_arguments={
                'slam': 'False',
                'namespace':                 ['/' + namespace],    
                'use_namespace':            'True',
                'map':                      '',
                'map_server':               'False',
                'params_file':              nav_params_file,
                'default_bt_xml_filename':  os.path.join(
                                                get_package_share_directory('nav2_bt_navigator'),
                                                'behavior_trees',
                                                'navigate_w_replanning_and_recovery.xml'
                                            ),
                'autostart':                'true',
                'use_sim_time':             use_sim_time,
                'log_level':                'warn'
            }.items()
        ))

    # ─── Initial Pose + RViz per robot ───
    for r in robots:
        ns   = '/' + r['name']
        pose = (
            '{header: {frame_id: map}, pose: '
            '{pose: {position: {x: ' + r['x'] +
            ', y: ' + r['y'] +
            ', z: ' + r['z'] +
            '}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}}}'
        )

        # publish initialpose
        ld.add_action(ExecuteProcess(
            cmd=[
                'ros2','topic','pub','-t','3','--qos-reliability','reliable',
                f'{ns}/initialpose',
                'geometry_msgs/PoseWithCovarianceStamped',
                pose
            ],
            output='screen'
        ))

        # then RViz (conditionally)
        ld.add_action(IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(nav_launch_dir, 'rviz_launch.py')),
            launch_arguments={
                'use_sim_time': use_sim_time,
                'namespace':    [ns],       
                'use_namespace':'True',
                'rviz_config':  rviz_config_file,
                'log_level':    'warn'
            }.items(),
            condition=IfCondition(enable_rviz)
        ))

    ld.add_action(
        actions.RegisterEventHandler(
            OnProcessExit(
                target_action=webots,  # or any important node like WebotsLauncher
                on_exit=[actions.EmitEvent(event=events.Shutdown())]
            )
        )
    )

    return ld