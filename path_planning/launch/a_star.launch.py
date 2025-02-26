import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution

def generate_launch_description():
    package_name = 'path_planning' 

    config_path = os.path.join(get_package_share_directory(package_name), 'config', 'map1.yaml')
    rviz_config_path = os.path.join(get_package_share_directory(package_name), 'rviz', 'basic_map.rviz')

    # RViz2 Node
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='log',
        arguments=['-d', rviz_config_path]
    )

    # GridMap Node
    grid_map_node = Node(
        package=package_name,
        executable='grid_map_node',  # Ensure this matches your compiled node name
        name='grid_map_node',
        output='screen',
        parameters=[{'config_file': config_path}]
    )

    # Planner
    plan_node = Node(
        package = package_name,
        executable = 'path_planner',
        name = 'a_star_planner',
        output = 'screen',
        parameters=[{'config_file': config_path}]
    )

    return LaunchDescription([
        grid_map_node,
        rviz_node,
        plan_node
    ])
