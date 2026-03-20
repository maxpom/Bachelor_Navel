"""Launch file für den Nav2-Stack (Lokalisierung + Navigation).

Startet alle Nav2-Lifecycle-Nodes in zwei Gruppen:
  - Lokalisierung:  map_server, amcl
  - Navigation:     controller_server, smoother_server, planner_server,
                    route_server, behavior_server, velocity_smoother,
                    collision_monitor, bt_navigator, waypoint_follower

Unterstützt sowohl normale Nodes (use_composition:=False) als auch
Composable Nodes in einem gemeinsamen Container (use_composition:=True).

Basiert auf dem offiziellen Nav2-Bringup-Launch-File:
  https://github.com/ros-navigation/navigation2/blob/main/nav2_bringup/launch/navigation_launch.py

Copyright (c) 2018 Intel Corporation – Apache License 2.0
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    GroupAction,
    LogInfo,
    SetEnvironmentVariable,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import LoadComposableNodes, Node, PushRosNamespace, SetParameter
from launch_ros.descriptions import ComposableNode, ParameterFile
from nav2_common.launch import RewrittenYaml


def generate_launch_description():
    """Erstellt die LaunchDescription für den vollständigen Nav2-Stack.

    Returns:
        LaunchDescription mit allen Nav2-Nodes, Lifecycle-Managern und
        optionalem Composable-Node-Container.
    """
    package_dir = get_package_share_directory('my_package')

    # Pfade zu Konfigurationsdateien.
    map_yaml = os.path.join(package_dir, 'config', 'map_zeki.yaml')
    rviz_config_path = os.path.join(package_dir, 'config', 'slam.rviz')
    config_dir = os.path.join(package_dir, 'config')
    config_file_default = 'nav2_params.yaml'

    # ---------------------------------------------------------------------------
    # LaunchConfiguration-Variablen
    # ---------------------------------------------------------------------------
    namespace = LaunchConfiguration('namespace')
    use_sim_time = LaunchConfiguration('use_sim_time')
    autostart = LaunchConfiguration('autostart')
    use_composition = LaunchConfiguration('use_composition')
    container_name = LaunchConfiguration('container_name')
    container_name_full = (namespace, '/', container_name)
    use_respawn = LaunchConfiguration('use_respawn')
    log_level = LaunchConfiguration('log_level')

    params_file = PathJoinSubstitution([
        config_dir,
        LaunchConfiguration('params_file'),
    ])

    # ---------------------------------------------------------------------------
    # Lifecycle-Node-Listen
    # ---------------------------------------------------------------------------
    lifecycle_nodes = [
        'map_server',
        'amcl',
        'controller_server',
        'smoother_server',
        'planner_server',
        'route_server',
        'behavior_server',
        'velocity_smoother',
        'collision_monitor',
        'bt_navigator',
        'waypoint_follower',
    ]
    localization_nodes = ['map_server', 'amcl']
    navigation_nodes = [n for n in lifecycle_nodes if n not in localization_nodes]

    remappings = [('/tf', 'tf'), ('/tf_static', 'tf_static')]

    # ---------------------------------------------------------------------------
    # Parameter-Substitutionen für namespace-abhängige Frame-IDs (AMCL).
    # ---------------------------------------------------------------------------
    param_substitutions = {
        'amcl.ros__parameters.base_frame_id': PythonExpression([
            "'", namespace, "' + '_base_link' if '", namespace, "' != '' else 'base_link'",
        ]),
        'amcl.ros__parameters.odom_frame_id': PythonExpression([
            "'", namespace, "' + '_odom' if '", namespace, "' != '' else 'odom'",
        ]),
        'amcl.ros__parameters.global_frame_id': PythonExpression([
            "'", namespace, "' + '_map' if '", namespace, "' != '' else 'map'",
        ]),
        'autostart': autostart,
    }

    configured_params = ParameterFile(
        RewrittenYaml(
            source_file=params_file,
            root_key=namespace,
            param_rewrites=param_substitutions,
            convert_types=True,
        ),
        allow_substs=True,
    )

    # ---------------------------------------------------------------------------
    # LaunchArgument-Deklarationen
    # ---------------------------------------------------------------------------
    declare_args = [
        DeclareLaunchArgument(
            'namespace',
            default_value='',
            description='Top-level namespace für alle Nav2-Nodes.',
        ),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='True',
            description='Simulationszeit verwenden (Webots/Gazebo).',
        ),
        DeclareLaunchArgument(
            'params_file',
            default_value=config_file_default,
            description='Name der Nav2-Parameterdatei (relativ zu config/).',
        ),
        DeclareLaunchArgument(
            'autostart',
            default_value='True',
            description='Nav2-Stack automatisch in den aktiven Zustand versetzen.',
        ),
        DeclareLaunchArgument(
            'use_composition',
            default_value='False',
            description='Composable Nodes in gemeinsamen Container laden.',
        ),
        DeclareLaunchArgument(
            'container_name',
            default_value='nav2_container',
            description='Name des Composable-Node-Containers.',
        ),
        DeclareLaunchArgument(
            'use_respawn',
            default_value='False',
            description='Nodes bei Absturz automatisch neu starten.',
        ),
        DeclareLaunchArgument(
            'log_level',
            default_value='info',
            description='Log-Level für alle Nav2-Nodes.',
        ),
    ]

    # ---------------------------------------------------------------------------
    # Normale Nodes (use_composition:=False)
    # ---------------------------------------------------------------------------
    load_nodes = GroupAction(
        condition=IfCondition(PythonExpression(['not ', use_composition])),
        actions=[
            SetParameter('use_sim_time', use_sim_time),
            Node(
                package='nav2_controller',
                executable='controller_server',
                output='screen',
                respawn=use_respawn,
                respawn_delay=2.0,
                parameters=[configured_params],
                arguments=['--ros-args', '--log-level', log_level],
                remappings=remappings + [('cmd_vel', 'cmd_vel_nav')],
            ),
            Node(
                package='nav2_smoother',
                executable='smoother_server',
                name='smoother_server',
                output='screen',
                respawn=use_respawn,
                respawn_delay=2.0,
                parameters=[configured_params],
                arguments=['--ros-args', '--log-level', log_level],
                remappings=remappings,
            ),
            # planner_server: 3 s Verzögerung, damit /clock und initiale TFs
            # verfügbar sind, bevor der erste Planungsaufruf eingeht.
            TimerAction(
                period=3.0,
                actions=[
                    Node(
                        package='nav2_planner',
                        executable='planner_server',
                        name='planner_server',
                        output='screen',
                        respawn=use_respawn,
                        respawn_delay=2.0,
                        parameters=[
                            configured_params,
                            {'use_sim_time': True},
                            {'transform_tolerance': 1.0},
                        ],
                        arguments=['--ros-args', '--log-level', log_level],
                        remappings=remappings,
                    ),
                ],
            ),
            Node(
                package='nav2_route',
                executable='route_server',
                name='route_server',
                output='screen',
                respawn=use_respawn,
                respawn_delay=2.0,
                parameters=[configured_params],
                arguments=['--ros-args', '--log-level', log_level],
                remappings=remappings,
            ),
            Node(
                package='nav2_behaviors',
                executable='behavior_server',
                name='behavior_server',
                output='screen',
                respawn=use_respawn,
                respawn_delay=2.0,
                parameters=[configured_params],
                arguments=['--ros-args', '--log-level', log_level],
                remappings=remappings + [('cmd_vel', 'cmd_vel_nav')],
            ),
            Node(
                package='nav2_bt_navigator',
                executable='bt_navigator',
                name='bt_navigator',
                output='screen',
                respawn=use_respawn,
                respawn_delay=2.0,
                parameters=[configured_params],
                arguments=['--ros-args', '--log-level', log_level],
                remappings=remappings,
            ),
            Node(
                package='nav2_waypoint_follower',
                executable='waypoint_follower',
                name='waypoint_follower',
                output='screen',
                respawn=use_respawn,
                respawn_delay=2.0,
                parameters=[configured_params],
                arguments=['--ros-args', '--log-level', log_level],
                remappings=remappings,
            ),
            Node(
                package='nav2_velocity_smoother',
                executable='velocity_smoother',
                name='velocity_smoother',
                output='screen',
                respawn=use_respawn,
                respawn_delay=2.0,
                parameters=[configured_params],
                arguments=['--ros-args', '--log-level', log_level],
                remappings=remappings + [('cmd_vel', 'cmd_vel_nav')],
            ),
            Node(
                package='nav2_collision_monitor',
                executable='collision_monitor',
                name='collision_monitor',
                output='screen',
                respawn=use_respawn,
                respawn_delay=2.0,
                parameters=[configured_params],
                arguments=['--ros-args', '--log-level', log_level],
                remappings=remappings,
            ),
            Node(
                package='nav2_map_server',
                executable='map_server',
                name='map_server',
                output='screen',
                parameters=[{'use_sim_time': True, 'yaml_filename': map_yaml}],
                arguments=['--ros-args', '--log-level', log_level],
            ),
            Node(
                package='nav2_amcl',
                executable='amcl',
                name='amcl',
                output='screen',
                respawn=use_respawn,
                respawn_delay=2.0,
                parameters=[
                    configured_params,
                    {'transform_tolerance': 2.0},
                    {'use_sim_time': True},
                    {'tf_broadcast': True},
                ],
                arguments=['--ros-args', '--log-level', log_level],
                remappings=remappings,
            ),
            # Lifecycle-Manager – Lokalisierung und Navigation getrennt verwalten,
            # damit ein Neustart von AMCL nicht den gesamten Stack stoppt.
            Node(
                package='nav2_lifecycle_manager',
                executable='lifecycle_manager',
                name='lifecycle_manager_localization',
                output='screen',
                arguments=['--ros-args', '--log-level', log_level],
                parameters=[
                    {'autostart': autostart},
                    {'node_names': localization_nodes},
                ],
            ),
            Node(
                package='nav2_lifecycle_manager',
                executable='lifecycle_manager',
                name='lifecycle_manager_navigation',
                output='screen',
                arguments=['--ros-args', '--log-level', log_level],
                parameters=[
                    {'autostart': autostart},
                    {'node_names': navigation_nodes},
                ],
            ),
            Node(
                package='rviz2',
                executable='rviz2',
                name='rviz2',
                output='screen',
                arguments=['-d', rviz_config_path],
                parameters=[{'use_sim_time': True}],
            ),
        ],
    )

    # ---------------------------------------------------------------------------
    # Composable Nodes (use_composition:=True)
    # ---------------------------------------------------------------------------
    load_composable_nodes = GroupAction(
        condition=IfCondition(use_composition),
        actions=[
            SetParameter('use_sim_time', use_sim_time),
            LoadComposableNodes(
                target_container=container_name_full,
                composable_node_descriptions=[
                    ComposableNode(
                        package='nav2_controller',
                        plugin='nav2_controller::ControllerServer',
                        name='controller_server',
                        parameters=[configured_params],
                        remappings=remappings + [('cmd_vel', 'cmd_vel_nav')],
                    ),
                    ComposableNode(
                        package='nav2_smoother',
                        plugin='nav2_smoother::SmootherServer',
                        name='smoother_server',
                        parameters=[configured_params],
                        remappings=remappings,
                    ),
                    ComposableNode(
                        package='nav2_planner',
                        plugin='nav2_planner::PlannerServer',
                        name='planner_server',
                        parameters=[configured_params],
                        remappings=remappings,
                    ),
                    ComposableNode(
                        package='nav2_route',
                        plugin='nav2_route::RouteServer',
                        name='route_server',
                        parameters=[configured_params],
                        remappings=remappings,
                    ),
                    ComposableNode(
                        package='nav2_behaviors',
                        plugin='behavior_server::BehaviorServer',
                        name='behavior_server',
                        parameters=[configured_params],
                        remappings=remappings + [('cmd_vel', 'cmd_vel_nav')],
                    ),
                    ComposableNode(
                        package='nav2_bt_navigator',
                        plugin='nav2_bt_navigator::BtNavigator',
                        name='bt_navigator',
                        parameters=[configured_params],
                        remappings=remappings,
                    ),
                    ComposableNode(
                        package='nav2_waypoint_follower',
                        plugin='nav2_waypoint_follower::WaypointFollower',
                        name='waypoint_follower',
                        parameters=[configured_params],
                        remappings=remappings,
                    ),
                    ComposableNode(
                        package='nav2_velocity_smoother',
                        plugin='nav2_velocity_smoother::VelocitySmoother',
                        name='velocity_smoother',
                        parameters=[configured_params],
                        remappings=remappings + [('cmd_vel', 'cmd_vel_nav')],
                    ),
                    ComposableNode(
                        package='nav2_collision_monitor',
                        plugin='nav2_collision_monitor::CollisionMonitor',
                        name='collision_monitor',
                        parameters=[configured_params],
                        remappings=remappings,
                    ),
                    ComposableNode(
                        package='nav2_map_server',
                        plugin='nav2_map_server::MapServer',
                        name='map_server',
                        parameters=[{'use_sim_time': True, 'yaml_filename': map_yaml}],
                        remappings=remappings,
                    ),
                    ComposableNode(
                        package='nav2_amcl',
                        plugin='nav2_amcl::AmclNode',
                        name='amcl',
                        parameters=[configured_params],
                        remappings=remappings,
                    ),
                    ComposableNode(
                        package='nav2_lifecycle_manager',
                        plugin='nav2_lifecycle_manager::LifecycleManager',
                        name='lifecycle_manager_localization',
                        parameters=[{
                            'autostart': autostart,
                            'node_names': localization_nodes,
                        }],
                    ),
                    ComposableNode(
                        package='nav2_lifecycle_manager',
                        plugin='nav2_lifecycle_manager::LifecycleManager',
                        name='lifecycle_manager_navigation',
                        parameters=[{
                            'autostart': autostart,
                            'node_names': navigation_nodes,
                        }],
                    ),
                ],
            ),
        ],
    )

    # ---------------------------------------------------------------------------
    # LaunchDescription zusammenbauen
    # ---------------------------------------------------------------------------
    ld = LaunchDescription()

    ld.add_action(declare_args[0])  # 'namespace'

    ld.add_action(SetEnvironmentVariable('RCUTILS_LOGGING_BUFFERED_STREAM', '1'))
    ld.add_action(PushRosNamespace(namespace=namespace))

    for arg in declare_args[1:]:
        ld.add_action(arg)

    ld.add_action(load_nodes)
    ld.add_action(load_composable_nodes)
    ld.add_action(LogInfo(msg=['Navigation launch – params file: ', params_file]))

    return ld