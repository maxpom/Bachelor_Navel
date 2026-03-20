source /opt/ros/jazzy/setup.bash
source install/local_setup.bash
ros2 launch my_package navigation_launch.py ${1:+params_file:=$1}
