
import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from ament_index_python.packages import get_package_share_directory
from webots_ros2_driver.webots_launcher import WebotsLauncher
from webots_ros2_driver.webots_controller import WebotsController
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription
from webots_ros2_driver.wait_for_controller_connection import WaitForControllerConnection
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory, get_packages_with_prefixes
import launch

package_dir = get_package_share_directory('couliglig_bot')

def fix_mesh_path_error():
    wbt_path = os.path.join(package_dir,'worlds','couliglig_bot.wbt')
    with open(wbt_path, "r") as file:
        content = file.read()

    newContent = content.replace("..",package_dir + '/stl_files')

    # rewrite the wbt content
    with open(wbt_path, "w") as file:
        file.write(newContent)


def generate_launch_description():
    world = LaunchConfiguration('world', default='couliglig_bot.wbt')
    mode = LaunchConfiguration('mode',default='realtime')
    use_nav = LaunchConfiguration('nav', default=False)
    use_slam = LaunchConfiguration('slam', default=False)
    use_sim_time = LaunchConfiguration('use_sim_time', default=True)
    # fix the mesh path error of wbt
    fix_mesh_path_error()


    # launch the instance of the webots
    webots = WebotsLauncher(
        world=os.path.join(package_dir,'worlds','couliglig_bot.wbt'),
        mode='realtime',
        ros2_supervisor=True
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': '<robot name=""><link name=""/></robot>'
        }],
    )

    footprint_publisher = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        output='screen',
        arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'base_footprint'],
    )


    # ROS control spawners
    # controller_manager_timeout = ['--controller-manager-timeout', '50']
    # controller_manager_prefix = 'python.exe' if os.name == 'nt' else ''
    # diffdrive_controller_spawner = Node(
    #     package='controller_manager',
    #     executable='spawner',
    #     output='screen',
    #     prefix=controller_manager_prefix,
    #     arguments=['diffdrive_controller'] + controller_manager_timeout,
    # )
    # joint_state_broadcaster_spawner = Node(
    #     package='controller_manager',
    #     executable='spawner',
    #     output='screen',
    #     prefix=controller_manager_prefix,
    #     arguments=['joint_state_broadcaster'] + controller_manager_timeout,
    # )
    # ros_control_spawners = [diffdrive_controller_spawner, joint_state_broadcaster_spawner]

    robot_description_path = os.path.join(package_dir, 'resource', 'couliglig_bot.urdf')
    ros2_control_params = os.path.join(package_dir, 'config', 'ros2control.yaml')
    mappings = [('/diffdrive_controller/cmd_vel_unstamped', '/cmd_vel'), ('/diffdrive_controller/odom', '/odom')]
    
    turtlebot_driver = WebotsController(
        robot_name='robot1',
        parameters=[
            {'robot_description': robot_description_path,
             'use_sim_time': use_sim_time,
             'update_rate': 50, 
             'set_robot_state_publisher': True},
            # ros2_control_params
        ],
        remappings=mappings,
        respawn=True
    )

    # Navigation
    navigation_nodes = []
    
    # SLAM
    if 'turtlebot3_cartographer' in get_packages_with_prefixes():
        turtlebot_slam = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(
                get_package_share_directory('turtlebot3_cartographer'), 'launch', 'cartographer.launch.py')),
            launch_arguments=[
                ('use_sim_time', use_sim_time),
            ],
            condition=launch.conditions.IfCondition(use_slam))
        navigation_nodes.append(turtlebot_slam)

    # Wait for the simulation to be ready to start navigation nodes
    waiting_nodes = WaitForControllerConnection(
        target_driver=turtlebot_driver,
        nodes_to_start=navigation_nodes
    )

    return LaunchDescription([

        webots,
        webots._supervisor,

        robot_state_publisher,
        footprint_publisher,
        
        turtlebot_driver,
        waiting_nodes
    ])


