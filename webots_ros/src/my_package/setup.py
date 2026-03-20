from setuptools import find_packages, setup

package_name = 'my_package'
data_files = []
data_files.append(('share/ament_index/resource_index/packages', ['resource/' + package_name]))
data_files.append(('share/' + package_name + '/launch', [
    'launch/robot_launch.py',
    'launch/navigation_launch.py',
    'launch/cartographer_launch.py',
    'launch/multiple_robot_launch.py',
    'launch/szenario_launch.py',
    ]))

data_files.append(('share/' + package_name + '/worlds', [
    'worlds/zeki.wbt',
    'worlds/zeki_multiple.wbt',
    'worlds/untitled6.obj',
    ]))

data_files.append(('share/' + package_name + '/resource', [
    'resource/my_robot.urdf',
    'resource/pedestrian.urdf',
    'resource/pedestrian2.urdf',
    'resource/pedestrian3.urdf',
    ]))

data_files.append(('share/' + package_name, ['package.xml']))
data_files.append(('share/' + package_name + '/config', [
    'config/navel_cartographer.lua',
    'config/slam.rviz',
    'config/room_waypoints.yaml',
    'config/nav_to_pose_w_replanning_and_recovery.xml',
    'config/nav_to_pose_w_replanning_and_recovery_base.xml',
]))

data_files.append(('share/' + package_name + '/config', [
    'config/nav/nav2_params.yaml',
    'config/nav/nav2_params_RRT.yaml',
    'config/nav/nav2_params_Bspline.yaml',
    'config/nav/nav2_params_MPC.yaml',
    'config/nav/nav2_params_Thesis.yaml',
    #'config/nav/nav2_params_MPC_RRT.yaml',
    'config/nav/nav2_params_MPC_Bspline.yaml',
    #'config/nav/nav2_params_RRT_Bspline.yaml',
]))

data_files.append(('share/' + package_name + '/config', [
    'map/map_zeki.yaml',
    'map/map_zeki.pgm',
]))


setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=data_files,
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='user',
    maintainer_email='user.name@mail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest', 'launch_testing'],
    entry_points={
        'console_scripts': [
            'navel_driver = my_package.navel_driver:main',
            'obstacle_avoider = my_package.obstacle_avoider:main',
            'odom_publisher = my_package.odom_publisher:main',
            'scan_republisher = my_package.scan_republisher:main',
            'pedestrian_driver = my_package.pedestrian_driver:main',
            'room_navigation = my_package.room_navigation:main',
            'motion_detector = my_package.motion_detection:main',
            'metric_logger = my_package.metric_logger:main',
        ],
    },
)