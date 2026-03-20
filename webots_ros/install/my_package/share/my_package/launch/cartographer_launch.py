"""Launch file für Cartographer SLAM und RViz2-Visualisierung.

Startet:
  - cartographer_node:               2D-SLAM auf Basis von LiDAR und Odometrie.
  - cartographer_occupancy_grid_node: Publiziert die Occupancy-Grid-Karte auf /map.
  - rviz2:                           Visualisierung mit vorkonfiguriertem Layout.

Konfiguration:
  - Lua-Konfiguration: config/navel_cartographer.lua
  - RViz-Layout:       config/mapping_only.rviz

Quellen:
  [CARTO_ROS2] Cartographer Authors (2024). ROS 2 Integration – Cartographer.
               https://google-cartographer-ros.readthedocs.io/en/latest/
  [NAV2_LAUNCH] Open Navigation LLC (2024). Writing Launch Files.
               https://docs.nav2.org/tutorials/docs/navigation2_with_slam.html
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    """Erstellt die LaunchDescription für Cartographer SLAM.

    Returns:
        LaunchDescription mit Cartographer-, OccupancyGrid- und RViz2-Node.
    """
    package_dir = get_package_share_directory('my_package')

    # Pfade zu Konfigurationsdateien.
    cartographer_config_dir = os.path.join(package_dir, 'config')
    rviz_config_path = os.path.join(package_dir, 'config', 'slam.rviz')

    # ---------------------------------------------------------------------------
    # Cartographer SLAM [CARTO_ROS2]
    # Liest LiDAR (/scan) und Odometrie (/odom) und erzeugt eine 2D-Karte.
    # ---------------------------------------------------------------------------
    cartographer_node = Node(
        package='cartographer_ros',
        executable='cartographer_node',
        name='cartographer_node',
        output='screen',
        parameters=[{'use_sim_time': True}],
        arguments=[
            '-configuration_directory', cartographer_config_dir,
            '-configuration_basename', 'navel_cartographer.lua',
        ],
        remappings=[('odom', '/odom')],
    )

    # ---------------------------------------------------------------------------
    # Occupancy Grid Publisher
    # Konvertiert die interne Cartographer-Karte in nav_msgs/OccupancyGrid (/map).
    # ---------------------------------------------------------------------------
    occupancy_grid_node = Node(
        package='cartographer_ros',
        executable='cartographer_occupancy_grid_node',
        name='occupancy_grid_node',
        output='screen',
        parameters=[
            {'use_sim_time': True},
            {'resolution': 0.05},
            {'publish_period_sec': 0.5},
        ],
    )

    # ---------------------------------------------------------------------------
    # RViz2 – Visualisierung
    # ---------------------------------------------------------------------------
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_path],
        parameters=[{'use_sim_time': True}],
    )

    return LaunchDescription([
        cartographer_node,
        occupancy_grid_node,
        rviz_node,
    ])