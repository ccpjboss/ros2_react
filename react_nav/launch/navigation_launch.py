from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, GroupAction
from launch.substitutions import TextSubstitution
from launch.substitutions import EnvironmentVariable, LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    map_dir = LaunchConfiguration(
        'map',
        default='/home/ccpjboss/ros2_test/src/ros2_react/react_nav/maps/1r5.yaml')

    params_dir = LaunchConfiguration(
        'params',
        default='/home/ccpjboss/ros2_test/src/ros2_react/react_nav/params/nav2_params.yaml')

    #nav2_launch_dir = os.path.join(get_package_share_directory('nav2_bringup'), 'launch')
    #nav2_launch_dir = "/home/ccpjboss/ros2_test/src/navigation2/nav2_bringup/launch"
    nav2_launch_dir = "/home/ccpjboss/ros2_test/src/navigation2/nav2_bringup/bringup/launch"

    rviz_config_dir = "/home/ccpjboss/ros2_test/src/ros2_react/react_nav/rviz/rviz_default.rviz"

    return LaunchDescription([
        DeclareLaunchArgument(
            'map',
            default_value=map_dir,
            description='Full path to map file to load'),

        DeclareLaunchArgument(
            'params',
            default_value=params_dir,
            description='Full path to param file to load'),

        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation (Gazebo) clock if true'),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([nav2_launch_dir, '/navigation_launch.py']),
            launch_arguments={
                'map': map_dir,
                'use_sim_time': use_sim_time,
                'autostart': 'true',
                'params_file': params_dir}.items(),
        ),
    ])
