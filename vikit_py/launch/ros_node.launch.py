from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    
    # Your parameter dictionary
    config = {
        'some_int': 5,
        'nested_group': {
            'some_string': 'hello'
        }
    }

    return LaunchDescription([
        Node(
            package='your_package_name',
            executable='your_executable_name',
            name='node_name',
            output='screen',
            parameters=[] # Pass dictionary directly here
        )
    ])