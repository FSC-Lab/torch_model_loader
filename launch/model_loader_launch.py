from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def launch_setup(context, *args, **kwargs):
    uav_prefix = LaunchConfiguration('uav_prefix').perform(context)

    # Get absolute path to the parameter file
    package_dir = get_package_share_directory('network_loader')
    param_file = os.path.join(package_dir, 'config', 'params.yaml')
    print(f"Using parameter file: {param_file}")
    return [
        #     Node(
        #     package='network_loader',
        #     executable='minimal_publisher_node',
        #     name='minimal_publisher',
        #     output='screen',
        #     parameters=[param_file],  # ✅ now it's a full path
        #     arguments=uav_list
        # ),
        Node(
            package='network_loader',
            executable='model_loader_node',
            name='model_loader',  # Changed to match params.yaml namespace
            namespace=uav_prefix,
            output='screen',
            parameters=[param_file],
        )
    ]

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'uav_prefix',
            default_value='uav_0',
            description='Namespace for this UAV instance'
        ),
        OpaqueFunction(function=launch_setup)
    ])