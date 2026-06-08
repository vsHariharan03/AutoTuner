from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    steve_navigation = get_package_share_directory('steve_navigation_autonav')
    nav2_bringup = get_package_share_directory('nav2_bringup')

    nav2_params = os.path.join(steve_navigation, 'config', 'nav2_params.yaml')
    bt_file = os.path.join(steve_navigation, 'behavior_trees', 'navigate_w_replanning_bt.xml')
    print(nav2_params)
    print(bt_file) 
    print(nav2_bringup)   

    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(nav2_bringup, 'launch', 'navigation_launch.py')
            ),
            launch_arguments={
                'use_sim_time': 'true',
                'params_file': nav2_params,
                'bt_xml_file': bt_file,
                'autostart': 'true',
                'use_sim_time' : 'true',
                'map_subscribe_transient_local': 'true',  # ensure static_layer subscribes correctly
            }.items()
        )
    ])
