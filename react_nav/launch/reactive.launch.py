from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import TextSubstitution
from launch.substitutions import EnvironmentVariable
import launch.actions

def generate_launch_description():
    use_sim_time = launch.substitutions.LaunchConfiguration('use_sim_time', default='true')
    return LaunchDescription([
        Node(
            package='stage_ros2',
            namespace='/',
            executable='stage_ros2',
            name='stage',
            parameters=[
                {"world":"./src/react_nav/worlds/rooms.world"}
            ],
            remappings=[
                ("/stage/ranger_0","/scan")
            ]
        ),
        Node(
            package='react_nav',
            namespace='/',
            executable='reactive',
            name='controller'
        ),
        Node(
            package='slam_gmapping',
            namespace='/',
            executable='slam_gmapping',
            output='screen',
            name='slam',
            parameters=[
                {'use_sim_time':use_sim_time}
            ]
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            output='screen',
            name='rviz'
        )
    ])