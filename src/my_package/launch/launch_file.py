import launch
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory, get_packages_with_prefixes
from launch.actions import IncludeLaunchDescription
import os

def generate_launch_description():

    slam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('my_package'), 'launch', 'custom_cartorapher.launch.py')),
        launch_arguments=[
            ('use_sim_time', 'false'),
        ],)
    
    return LaunchDescription([
        # Remapping the depth image topic and camera info topic
        DeclareLaunchArgument('depth_topic', default_value='/zed/zed_node/depth/depth_registered', description='Depth image topic'),
        DeclareLaunchArgument('camera_info_topic', default_value='/zed/zed_node/depth/camera_info', description='Camera info topic'),
        
        # Launch the depthimage_to_laserscan node
        Node(
            package='depthimage_to_laserscan',
            executable='depthimage_to_laserscan_node',
            name='depthimage_to_laserscan',
            output='screen',
            parameters=[{
                'scan_height': 1,
                'scan_time': 0.033,
                'range_min': 0.45,
                'range_max': 10.0,
                'output_frame_id': 'camera_depth_frame'
            }],
            remappings=[
                ('depth', '/zed/zed_node/depth/depth_registered'),
                ('depth_camera_info', '/zed/zed_node/depth/camera_info'),
                ('scan', '/scan')
            ]
        ),

        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_tf_base_to_imu',
            arguments=['0', '0', '0', '0', '0', '0', 'zed_camera_link', 'imu_link'],
            output='screen'
        ),

        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='baselink_camearadeptframe_static_pubisher',
            output='screen',
            arguments=['0', '0', '0', '0', '0', '0', 'zed_camera_link', 'camera_depth_frame'],
        ),



        slam
    ])
