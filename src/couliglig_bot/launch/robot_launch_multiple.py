
import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from ament_index_python.packages import get_package_share_directory
from webots_ros2_driver.webots_launcher import WebotsLauncher
from webots_ros2_driver.webots_controller import WebotsController
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription, OpaqueFunction
from webots_ros2_driver.wait_for_controller_connection import WaitForControllerConnection
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory, get_packages_with_prefixes
import launch

package_dir = get_package_share_directory('couliglig_bot')


def launch_setup(context, *args, **kwargs):
    namespace_value = LaunchConfiguration('namespace').perform(context)
    use_sim_time_value = LaunchConfiguration('use_sim_time').perform(context)

    urdf = os.path.join(package_dir, 'resource', 'couliglig_multiple_bot.urdf')

    remappings = [('/tf', 'tf'), ('/tf_static', 'tf_static')]

    nodes = []

    nodes.append(Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        namespace=namespace_value,
        output='screen',
        parameters=[{'robot_description': '<robot name=""><link name=""/></robot>'}],
        remappings=remappings,
    ))

    nodes.append(Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        namespace=namespace_value,
        output='screen',
        arguments=['0', '0', '0', '0', '0', '0', 'base_footprint', 'base_link'],
        remappings=remappings,
    ))

    nodes.append(WebotsController(
        robot_name=namespace_value,
        namespace=namespace_value,
        parameters=[
            {'robot_description': urdf,
             'use_sim_time': use_sim_time_value,
             'set_robot_state_publisher': True,
             'update_rate': 50},
        ],
        remappings=remappings,
        respawn=True,
    ))

    return nodes

def generate_launch_description():

    declare_namespace_cmd = DeclareLaunchArgument(
        'namespace',
        default_value='robot',
        description='Namespace of the robot target',
    )

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation time',
    )

    return LaunchDescription([
        declare_namespace_cmd,
        declare_use_sim_time_cmd,
        OpaqueFunction(function=launch_setup),
    ])

