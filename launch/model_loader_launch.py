from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def launch_setup(context, *args, **kwargs):
    objects_arg = LaunchConfiguration('objects').perform(context)
    object_list = objects_arg.split()

    # Get absolute path to the parameter file
    package_dir = get_package_share_directory('network_loader')
    param_file = os.path.join(package_dir, 'config', 'params.yaml')
    print(f"Using parameter file: {param_file}")
    return [
            Node(
            package='network_loader',
            executable='minimal_publisher_node',
            name='minimal_publisher',
            output='screen',
            parameters=[param_file],  # ✅ now it's a full path
            arguments=object_list
        ),
        Node(
            package='network_loader',
            executable='model_loader_node',
            name='model_loader',  # Changed to match params.yaml namespace
            output='screen',
            parameters=[param_file],
            arguments=object_list
        )
    ]

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'objects',
            default_value='x500_0',
            description='List of objects to track'
        ),
        OpaqueFunction(function=launch_setup)
    ])