# szenario_launch.py
# Startet die Szenario-Nodes fuer das Navel-Besucherfuehrungssystem:
#   - motion_detector: Kamerabasierte Bewegungserkennung.
#   - room_navigation: Waypoint-Guidance-Supervisor.

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    """Erzeugt die LaunchDescription fuer das Fuehrungsszenario.

    Returns:
        LaunchDescription mit motion_detector- und room_navigation-Node.
    """
    motion_detector_node = Node(
        package='my_package',
        executable='motion_detector',
        name='motion_detector',
        output='screen',
        parameters=[{'use_sim_time': True}],
    )

    room_navigation_node = Node(
        package='my_package',
        executable='room_navigation',
        name='room_navigation',
        output='screen',
        parameters=[{'use_sim_time': True}],
    )

    return LaunchDescription([
        motion_detector_node,
        room_navigation_node,
    ])