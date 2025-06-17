import os
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node, PushRosNamespace
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    # Define paths to config files
    home_dir = os.path.expanduser('~')
    identity_config_path = os.path.join(home_dir, 'ros2_config', 'robot_identity.yaml')
    camera_tuning_config = os.path.join(home_dir, 'ros2_config', 'camera_tuning.yaml')
    apriltag_tuning_config = os.path.join(home_dir, 'ros2_config', 'apriltag_tuning.yaml')

    # Load the identity config to get the camera name and frame_id
    with open(identity_config_path, 'r') as f:
        identity_config = yaml.safe_load(f)
        camera_name = identity_config['vision_system']['ros__parameters']['camera_name']
        camera_frame_id = identity_config['vision_system']['ros__parameters']['camera_frame_id']

    # Create a group of actions that will all be pushed into the same namespace
    namespaced_group = GroupAction(
        actions=[
            # Push all nodes into a namespace defined by camera_name
            PushRosNamespace(camera_name),

            # 1. Camera Driver Node
            Node(
                package='spinnaker_camera_driver',
                executable='camera_driver_node',
                name='spinnaker_camera_node',
                parameters=[
                    camera_tuning_config,
                    {'frame_id': camera_frame_id} # Set the frame_id directly
                ],
                output='screen'
            ),

            # 2. Image Processing Node for Undistortion
            # Note: topic remappings are relative to the namespace
            Node(
                package='image_proc',
                executable='image_proc',
                name='image_proc_node',
                remappings=[
                    ('image', 'spinnaker_camera_node/image_raw'),
                    ('image_raw', 'image_proc_node/image_raw_out'), # Avoid topic collision
                    ('image_rect', 'image_rect'), # Output topic
                ],
                output='screen'
            ),

            # 3. AprilTag Detection Node
            Node(
                package='apriltag_ros',
                executable='apriltag_node',
                name='apriltag_node',
                parameters=[apriltag_tuning_config],
                output='screen',
                remappings=[
                    # Subscribe to the *rectified* image from image_proc
                    ('image_rect', 'image_rect'),
                    ('camera_info', 'spinnaker_camera_node/camera_info'),
                ]
            ),
        ]
    )

    return LaunchDescription([namespaced_group])
