# NavelTrajectory

## Installation:

Ändern von allen config files (nav2_params) bt_navigator:
default_nav_to_pose_bt_xml: "Pfad zu nav_to_pose_w_replanning_and_recovery.xml"

## Coding-Style
https://google.github.io/styleguide/cppguide.html#Comments

## Usage:

1. cd /webots_ros
2. ./start.sh
3. ./navigation.sh
4. ./test.sh Testfall

### Test 1: Einfacher Go-To-Pose Test Schwierigkeit besteht in initialer drehung
1. cd /webots_ros
2. ./start.sh
3. ./navigation.sh configfile(standard=nav2_params.yaml)
4. ./test.sh test_1 "name"

### Test 2: Besucher Szenario mit einem Zwischenstop und initialer Drehung
#### The results are split for every waypoint
1. cd /webots_ros
2. ./start.sh
3. ./navigation.sh configfile(standard=nav2_params.yaml)
4. ./test.sh test_2 "name"
5. At the start and at every Checkpoint the Navel needs to see movement(checking if still followed)
    Do this by either controlling the Pedestrian via :
    ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args --remap cmd_vel:=person_1/cmd_vel
    Or just dragging it in the Simulation.

### Test 3: Dynmische Hindernisse
#### mit erstem Roboter sollte Konfrontation geschehen
1. cd /webots_ros
2. ./start_multiple.sh
3. ./navigation.sh configfile(standard=nav2_params.yaml)
4. ./test.sh test_3 "name"


