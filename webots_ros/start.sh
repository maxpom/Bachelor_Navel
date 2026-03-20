source /opt/ros/jazzy/setup.bash

#cleanup
pkill -f ros2
pkill -f webots
rm -rf build/ install/ log/

colcon build --packages-select my_package rrtstar_planner bspline_smoother
source install/local_setup.bash
export WEBOTS_CONTROLLER_PATH="$PWD/install/my_package/share/my_package/controllers"

ros2 launch my_package robot_launch.py


