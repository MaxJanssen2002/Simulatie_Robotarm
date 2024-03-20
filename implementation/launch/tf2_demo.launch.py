from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node


def generate_launch_description():
    ld = LaunchDescription([
        # Node(
        #     package='implementation',
        #     executable='static_tf2_broadcaster',
        #     name='broadcaster1',
        #     parameters=[
        #         {'turtlename': 'turtle1'}
        #     ]
        # ),
        # Node(
        #     package='implementation',
        #     executable='dynamic_tf2_broadcaster',
        #     name='broadcaster',
        # ),
        # Node(
        #     package='implementation',
        #     executable='tf2_listener',
        #     name='listener',
        # ),
        Node(
            package='implementation',
            executable='state_publisher',
            name='broadcaster',
        ),
    ])

    return ld