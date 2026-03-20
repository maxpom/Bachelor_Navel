"""Launch file für Mehroboter-Testszenarien in Webots.

Startet die Webots-Simulation (zeki_multiple.wbt) mit:
  - Navel als Hauptroboter
  - Drei Fußgänger-Controller (person_1 – person_3)
  - robot_state_publisher, odom_publisher, scan_republisher
  - obstacle_avoider je Fußgänger
  - motion_detector, room_navigation
  
"""

import os

import launch
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from webots_ros2_driver.webots_controller import WebotsController
from webots_ros2_driver.webots_launcher import WebotsLauncher


def generate_launch_description():
    """Erstellt die LaunchDescription für das Mehroboter-Szenario.

    Returns:
        LaunchDescription mit Webots, allen Controllern und Hilfs-Nodes.
    """
    package_dir = get_package_share_directory('my_package')

    # Pfade zu Ressourcen.
    robot_description_path = os.path.join(
        package_dir, 'resource', 'my_robot.urdf')
    pedestrian_description_paths = {
        'person_1': os.path.join(package_dir, 'resource', 'pedestrian.urdf'),
        'person_2': os.path.join(package_dir, 'resource', 'pedestrian2.urdf'),
        'person_3': os.path.join(package_dir, 'resource', 'pedestrian3.urdf'),
    }
    webots_world = os.path.join(package_dir, 'worlds', 'zeki_multiple.wbt')

    with open(robot_description_path, 'r', encoding='utf-8') as f:
        robot_desc = f.read()

    # ---------------------------------------------------------------------------
    # Webots
    # ---------------------------------------------------------------------------
    webots = WebotsLauncher(
        world=webots_world,
        ros2_supervisor=True,
    )

    # ---------------------------------------------------------------------------
    # WebotsController – Hauptroboter
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

    # ---------------------------------------------------------------------------
    # WebotsController – Fußgänger
    # Remappings bringen alle Topics in den jeweiligen Namespace.
    # ---------------------------------------------------------------------------
    def make_pedestrian_controller(name):
        """Erstellt einen WebotsController für einen Fußgänger.

        Args:
            name: Robot-Name im .wbt-File, gleichzeitig ROS2-Namespace.

        Returns:
            Konfigurierter WebotsController.
        """
        return WebotsController(
            robot_name=name,
            namespace=name,
            parameters=[
                {'robot_description': pedestrian_description_paths[name]},
                {'use_sim_time': True},
            ],
            remappings=[
                ('/lidar_left',                  f'/{name}/lidar_left'),
                ('/lidar_right',                 f'/{name}/lidar_right'),
                ('/joint_states',                f'/{name}/joint_states'),
                ('/head_camera/image_color',     f'/{name}/head_camera/image_color'),
            ],
        )

    pedestrian_1 = make_pedestrian_controller('person_1')
    pedestrian_2 = make_pedestrian_controller('person_2')
    pedestrian_3 = make_pedestrian_controller('person_3')

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

    motion_detector = Node(
        package='my_package',
        executable='motion_detector',
        parameters=[{'use_sim_time': True}],
    )

    room_navigation = Node(
        package='my_package',
        executable='room_navigation',
        parameters=[{'use_sim_time': True}],
    )

    # ---------------------------------------------------------------------------
    # obstacle_avoider – je ein Node pro Fußgänger-Namespace
    # ---------------------------------------------------------------------------
    obstacle_avoiders = [
        Node(
            package='my_package',
            executable='obstacle_avoider',
            namespace=name,
            parameters=[{'use_sim_time': True}],
        )
        for name in ('person_1', 'person_2', 'person_3')
    ]

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
        webots,
        webots._supervisor,
        my_robot_driver,
        robot_state_publisher,
        odom_publisher,
        scan_republisher,
        pedestrian_1,
        pedestrian_2,
        pedestrian_3,
        *obstacle_avoiders,
        motion_detector,
        room_navigation,
        on_webots_exit,
    ])