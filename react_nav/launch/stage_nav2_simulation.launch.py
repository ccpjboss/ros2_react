import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node

def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    map_dir = LaunchConfiguration(
        'map',
        default='/home/ccpjboss/ros2_test/src/ros2_react/react_nav/maps/1r5.yaml')

    params_dir = LaunchConfiguration(
        'params',
        default='/home/ccpjboss/ros2_test/src/ros2_react/react_nav/params/nav2_params.yaml')

    #nav2_launch_dir = os.path.join(get_package_share_directory('nav2_bringup'), 'launch')
    nav2_launch_dir = "/home/ccpjboss/ros2_test/src/navigation2/nav2_bringup/bringup/launch"

    react_launch_dir = os.path.join(get_package_share_directory('react_nav'), 'launch')

    rviz_config_dir = "/home/ccpjboss/ros2_test/src/ros2_react/react_nav/rviz/rviz_default.rviz"

    declare_map = DeclareLaunchArgument(
        'map',
        default_value=map_dir,
        description='Full path to map file to load')

    declare_params = DeclareLaunchArgument(
        'params',
        default_value=params_dir,
        description='Full path to param file to load')

    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true')

    bringup_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([react_launch_dir, '/bringup.launch.py']),
        launch_arguments={
            'map': map_dir,
            'use_sim_time': use_sim_time,
            'autostart': 'true',
            'params_file': params_dir}.items(),
    )

    navigation_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([react_launch_dir, '/navigation_launch.py']),
        launch_arguments={
            'map': map_dir,
            'use_sim_time': use_sim_time,
            'autostart': 'true',
            'params_file': params_dir}.items(),
    )

    localization_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([react_launch_dir, '/localization.launch.py']),
        launch_arguments={
            'map': map_dir,
            'use_sim_time': use_sim_time,
            'autostart': 'true',
            'params_file': params_dir}.items(),
    )
    
    navigation_cmd_timer = TimerAction(period=3.0,actions=[navigation_cmd])
    localization_cmd_timer = TimerAction(period=6.0,actions=[localization_cmd])
    
    ld = LaunchDescription()
    ld.add_action(declare_map)
    ld.add_action(declare_params)
    ld.add_action(declare_use_sim_time)
    ld.add_action(bringup_cmd)
    ld.add_action(navigation_cmd_timer)
    ld.add_action(localization_cmd_timer)

    return ld
