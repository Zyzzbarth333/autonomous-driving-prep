from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='sensor_publisher',
            executable='lidar_simulator',
            name='lidar_simulator',
            output='screen'
        ),
        Node(
            package='sensor_publisher',
            executable='obstacle_avoider',
            name='obstacle_avoider',
            output='screen'
        ),
        Node(
            package='sensor_publisher',
            executable='lidar_processor',
            name='lidar_processor',
            output='screen'
        ),
    ])