import os
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node, PushRosNamespace
from launch.actions import GroupAction

def generate_launch_description():
    # --- Corrected: Find config files within the package's share directory ---
    # This is the standard ROS 2 way to find package data files.
    # It requires the 'config' directory to be installed with the package.
    # (See notes on CMakeLists.txt modification below)
    robot_launch_share_dir = get_package_share_directory('robot_launch')
    
    identity_config_path = os.path.join(robot_launch_share_dir, 'config', 'robot_identity.yaml')
    camera_tuning_config = os.path.join(robot_launch_share_dir, 'config', 'camera_tuning.yaml')
    apriltag_tuning_config = os.path.join(robot_launch_share_dir, 'config', 'apriltag_tuning.yaml')

    # Load the identity config to get the camera name for the namespace
    # This is a valid use case for reading a file at launch-time, as the namespace
    # itself depends on the content.
    try:
        with open(identity_config_path, 'r') as f:
            identity_config = yaml.safe_load(f)
            # Safely get nested parameters
            ros_params = identity_config.get('vision_system', {}).get('ros__parameters', {})
            camera_name = ros_params.get('camera_name')
            camera_frame_id = ros_params.get('camera_frame_id')

            if not camera_name or not camera_frame_id:
                raise ValueError("'camera_name' or 'camera_frame_id' not found in robot_identity.yaml")

    except (IOError, yaml.YAMLError, ValueError) as e:
        # Provide a clear error message if the config is missing or malformed
        print(f"Error loading or parsing '{identity_config_path}': {e}")
        # You might want to return an empty LaunchDescription or exit
        return LaunchDescription([])

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