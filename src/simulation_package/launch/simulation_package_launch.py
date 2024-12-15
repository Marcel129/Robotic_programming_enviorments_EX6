from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # src_dir = '/home/marcel/studies/2_sem_mgr/RPE_lab/EX6/ros2_ws/src/simulation_package/config'
    return LaunchDescription([
        Node(
            package='simulation_package',
            executable='robot_controller'
        ),
        Node(
            package='simulation_package',
            executable='robot_simulator'
        )
        
    ])