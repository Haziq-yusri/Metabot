"""Launch the RobStride Python driver node with parameters from YAML."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    params_file = PathJoinSubstitution([
        FindPackageShare('robstride_control_py'),
        'config',
        'robstride.yaml',
    ])

    return LaunchDescription([
        DeclareLaunchArgument(
            'params_file',
            default_value=params_file,
            description='Path to the parameter YAML file',
        ),
        Node(
            package='robstride_control_py',
            executable='robstride_py_node',
            name='robstride_py_node',
            output='screen',
            parameters=[LaunchConfiguration('params_file')],
        ),
    ])
