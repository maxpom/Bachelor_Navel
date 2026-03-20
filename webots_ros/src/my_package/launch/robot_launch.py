"""Launch file for Webots simulation with Nav2-compatible robot setup.

Startet die Webots-Simulation (Navel) gemeinsam mit:
  - Ros2Supervisor (Webots-seitige Lifecycle-Verwaltung)
  - WebotsController für Hauptroboter und Fußgänger
  - robot_state_publisher
  - odom_publisher, scan_republisher (custom nodes)

Quelle Launch-API:
  https://docs.ros.org/en/jazzy/Tutorials/Intermediate/Launch/Launch-Main.html
"""

import os
import launch
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from webots_ros2_driver.webots_controller import WebotsController
from webots_ros2_driver.webots_launcher import WebotsLauncher


def generate_launch_description():
    """Erstellt die LaunchDescription für die Webots-Simulation.

    Returns:
        LaunchDescription mit allen Nodes und Event-Handlern.
    """
    package_dir = get_package_share_directory('my_package')

    # Pfade zu Ressourcen.
    robot_description_path = os.path.join(
        package_dir, 'resource', 'my_robot.urdf')
    pedestrian_description_path = os.path.join(
        package_dir, 'resource', 'pedestrian.urdf')
    webots_world = os.path.join(package_dir, 'worlds', 'zeki.wbt')

    with open(robot_description_path, 'r', encoding='utf-8') as f:
        robot_desc = f.read()

    # ---------------------------------------------------------------------------
    # Webots
    # ---------------------------------------------------------------------------
    declare_world_file_cmd = DeclareLaunchArgument(
        'world_file',
        default_value=webots_world,
        description='Vollständiger Pfad zur Webots-.wbt-Datei.',
    )

    # WebotsLauncher benötigt einen aufgelösten Pfad zur Parse-Zeit (os.stat).
    webots = WebotsLauncher(
        world=webots_world,
        ros2_supervisor=True,
    )

    # ---------------------------------------------------------------------------
    # WebotsController – Hauptroboter und Fußgänger
    # ---------------------------------------------------------------------------
    my_robot_driver = WebotsController(
        robot_name='Navel',
        parameters=[
            {'robot_description': robot_description_path},
            {'use_sim_time': True},
            {'publish_joints': True},
            {'publish_tf': False},  # TF wird vom odom_publisher veröffentlicht.
        ],
    )

    pedestrian_1 = WebotsController(
        robot_name='person_1',
        namespace='person_1',
        parameters=[
            {'robot_description': pedestrian_description_path},
            {'use_sim_time': True},
        ],
        remappings=[
            ('/lidar_left',  '/person_1/lidar_left'),
            ('/lidar_right', '/person_1/lidar_right'),
            ('/head_camera/image_color', '/person_1/head_camera/image_color'),
        ],
    )

    # ---------------------------------------------------------------------------
    # Standard-ROS2-Nodes
    # ---------------------------------------------------------------------------
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        emulate_tty=True,
        arguments=[robot_description_path],
        parameters=[
            {'use_sim_time': True},
            {'robot_description': robot_desc},
        ],
    )

    odom_publisher = Node(
        package='my_package',
        executable='odom_publisher',
        parameters=[{'use_sim_time': True}],
    )

    scan_republisher = Node(
        package='my_package',
        executable='scan_republisher',
        parameters=[{'use_sim_time': True}],
    )

    # ---------------------------------------------------------------------------
    # Shutdown-Handler: beendet den gesamten Launch, wenn Webots sich schließt.
    # ---------------------------------------------------------------------------
    on_webots_exit = launch.actions.RegisterEventHandler(
        event_handler=launch.event_handlers.OnProcessExit(
            target_action=webots,
            on_exit=[launch.actions.EmitEvent(
                event=launch.events.Shutdown())],
        )
    )

    return LaunchDescription([
        declare_world_file_cmd,
        webots,
        webots._supervisor,
        my_robot_driver,
        robot_state_publisher,
        odom_publisher,
        scan_republisher,
        pedestrian_1,
        on_webots_exit,
    ])