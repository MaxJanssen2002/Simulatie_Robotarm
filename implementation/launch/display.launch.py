import os
import launch_ros
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory



def generate_launch_description():

    use_sim_time = LaunchConfiguration('use_sim_time', default='false')

    pkg_share = launch_ros.substitutions.FindPackageShare(package='implementation').find('implementation')

    default_rviz_config_path = os.path.join(pkg_share, 'rviz/urdf.rviz')

    robot_arm_urdf = os.path.join(get_package_share_directory('implementation'), 'urdf/lynxmotion_arm.urdf')
    cup_urdf = os.path.join(get_package_share_directory('implementation'), 'urdf/cup.urdf')

    with open(robot_arm_urdf, 'r') as infp:
        robot_arm_description = infp.read()

    with open(cup_urdf, 'r') as infp:
        cup_description = infp.read()

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false', description='Use simulation (Gazebo) clock if true'),
        DeclareLaunchArgument(name='rvizconfig', default_value=default_rviz_config_path, description='Absolute path to rviz config file'),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher1',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time, 'robot_arm': robot_arm_description}],
            arguments=[robot_arm_urdf]
            ),
         Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher2',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time, 'cup': cup_description}],
            arguments=[cup_urdf]
            ),
        Node(
            package='implementation',
            executable='state_publisher',
            name='state_publisher',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}],
            
            ),
        Node(
            package='implementation',
            executable='cup_simulator',
            name='cup_simulator',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}],
           
            ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', LaunchConfiguration('rvizconfig')],
            ),
    ])