
import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from ament_index_python.packages import get_package_share_directory
from webots_ros2_driver.webots_launcher import WebotsLauncher
from webots_ros2_driver.webots_controller import WebotsController
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription, ExecuteProcess
from webots_ros2_driver.wait_for_controller_connection import WaitForControllerConnection
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory, get_packages_with_prefixes
import launch

package_dir = get_package_share_directory('couliglig_bot')
turtlebot3_multi_robot_dir = get_package_share_directory('turtlebot3_multi_robot')

def fix_mesh_path_error():
    wbt_path = os.path.join(package_dir,'worlds','couliglig_bot.wbt')
    with open(wbt_path, "r") as file:
        content = file.read()

    newContent = content.replace("..",package_dir + '/stl_files')

    # rewrite the wbt content
    with open(wbt_path, "w") as file:
        file.write(newContent)


def generate_launch_description():
    use_nav = LaunchConfiguration('nav', default=False)
    use_slam = LaunchConfiguration('slam', default=False)
    use_sim_time = LaunchConfiguration('use_sim_time', default=True)
    namespace = 'robot1'
    # fix the mesh path error of wbt
    fix_mesh_path_error()

    ld = LaunchDescription()
    ros2_control_params = os.path.join(package_dir, 'config', 'ros2control.yaml')
    mappings = [('/diffdrive_controller/cmd_vel_unstamped', '/cmd_vel'), ('/diffdrive_controller/odom', '/odom')]

    # Remappings TODO:
    remappings = [('/tf', 'tf'),
                  ('/tf_static', 'tf_static')]

    urdf = os.path.join(
        package_dir, 'resource', 'couliglig_bot.urdf'
    )   
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        namespace=namespace,
        output='screen',
        parameters=[{
            'robot_description': '<robot name=""><link name=""/></robot>',
        }],
        remappings=remappings,
    )
    footprint_publisher = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        namespace=namespace,
        output='screen',
        arguments=['0', '0', '0', '0', '0', '0','base_footprint','base_link'],
        remappings=remappings,
    )

    # Create spawn call
    turtlebot_driver = WebotsController(
        robot_name='robot1',
        namespace=namespace,
        parameters=[
            {'robot_description': urdf,
             'use_sim_time': use_sim_time,
             'set_robot_state_publisher': True,
             'update_rate': 50,  
             },
            #  ros2_control_params
        ],
        remappings=[*remappings],
        respawn=True
    )

    # ld.add_action(turtlebot_state_publisher)
    ld.add_action(robot_state_publisher)
    ld.add_action(footprint_publisher)
    ld.add_action(turtlebot_driver)

    return ld

