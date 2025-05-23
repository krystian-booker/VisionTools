import yaml
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node, ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory

def launch_setup(context, *args, **kwargs):
    camera_config_file_path = LaunchConfiguration('camera_config_file').perform(context)
    
    with open(camera_config_file_path, 'r') as f:
        config_data = yaml.safe_load(f)

    composable_nodes = []
    cameras = config_data.get('cameras', [])

    for camera_info in cameras:
        name = camera_info['name']
        serial_number = camera_info['serial_number']
        camera_type = camera_info['camera_type']
        ros_parameters = camera_info.get('ros_parameters', {})

        if 'parameter_file' in camera_info:
            parameter_file = camera_info['parameter_file']
        else:
            parameter_file = PathJoinSubstitution([
                FindPackageShare('spinnaker_camera_driver'),
                'config',
                camera_type + '.yaml'
            ]).perform(context)

        ros_parameters['serial_number'] = serial_number
        ros_parameters['parameter_file'] = parameter_file
        
        composable_nodes.append(
            ComposableNode(
                package='spinnaker_camera_driver',
                plugin='spinnaker_camera_driver::CameraDriver',
                name=name,
                parameters=[ros_parameters],
                remappings=[('~/control', '/exposure_control/control')],
                extra_arguments=[{'use_intra_process_comms': True}]
            )
        )

    if composable_nodes:
        container = ComposableNodeContainer(
            name='flir_camera_container',
            namespace='',
            package='rclcpp_components',
            executable='component_container',
            composable_node_descriptions=composable_nodes,
            output='screen'
        )
        return [container]
    
    return []

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'camera_config_file',
            description='Path to the YAML file containing camera configurations.'
        ),
        OpaqueFunction(function=launch_setup)
    ])
