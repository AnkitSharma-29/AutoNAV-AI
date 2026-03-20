import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from webots_ros2_driver.webots_launcher import WebotsLauncher
from webots_ros2_driver.webots_controller import WebotsController


def generate_launch_description():
    package_dir = get_package_share_directory('autonav_ros2')
    
    # Path to Webots World
    world_path = os.path.join(get_package_share_directory('autonav_ros2'), 'worlds', 'underground_tunnel_v2.wbt')
    
    # Webots Driver
    webots_driver = WebotsLauncher(
        world=world_path,
        mode='realtime'
    )
    
    # Webots ROS2 Bridge Controller for Mavic2Pro
    drone_driver = WebotsController(
        robot_name='Mavic2Pro',
        parameters=[
            {'robot_description': world_path}
        ],
        respawn=True
    )
    
    return LaunchDescription([
        webots,
        drone_driver,
        webots._supervisor,
    ])
