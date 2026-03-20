#!/usr/bin/env bash
source /opt/ros/jazzy/setup.bash
source install/local_setup.bash

CONFIG_LABEL="${2:-unnamed}"
CSV_PATH="test/${1}_results.csv"

cleanup() {
    if [ -n "${LOGGER_PID:-}" ] && kill -0 "$LOGGER_PID" 2>/dev/null; then
        kill "$LOGGER_PID" 2>/dev/null || true
        wait "$LOGGER_PID" 2>/dev/null || true
    fi
    echo ""
    if [ -f "$CSV_PATH" ]; then
        echo "=== Ergebnisse ==="
        cat "$CSV_PATH"
    fi
}
trap cleanup EXIT

set_initial_pose() {
    python3 -c "
import rclpy, math, time
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped
rclpy.init()
n = Node('ip')
n.set_parameters([rclpy.parameter.Parameter('use_sim_time', rclpy.Parameter.Type.BOOL, True)])
p = n.create_publisher(PoseWithCovarianceStamped, '/initialpose', 1)
time.sleep(1.0)
m = PoseWithCovarianceStamped()
m.header.frame_id = 'map'
m.header.stamp = n.get_clock().now().to_msg()
m.pose.pose.position.x = -1.883
m.pose.pose.orientation.z = math.sin(-1.5708/2)
m.pose.pose.orientation.w = math.cos(1.5708/2)
m.pose.covariance[0] = 0.25
m.pose.covariance[7] = 0.25
m.pose.covariance[35] = 0.07
p.publish(m)
time.sleep(0.5)
n.destroy_node()
rclpy.shutdown()
    "
    echo "Initial-Pose gesetzt. Warte auf AMCL-Konvergenz..."
    sleep 3
}


start_experiment() {
    local GOAL_X="$1"
    local GOAL_Y="$2"
    local GOAL_OZ="$3"
    local GOAL_OW="$4"

    echo "Sende /start_experiment Signal..."
    ros2 topic pub --times 1 -w 0 /start_experiment std_msgs/msg/Empty "{}" &

    # Goal direkt danach senden (gleicher Zeitpunkt)
    ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose "{
        pose: {
            header: {frame_id: 'map'},
            pose: {
                position: {x: ${GOAL_X}, y: ${GOAL_Y}, z: 0.0},
                orientation: {x: 0.0, y: 0.0, z: ${GOAL_OZ}, w: ${GOAL_OW}}
            }
        }
    }" --feedback
}

# -- 1) Logger ZUERST starten --
ros2 run my_package metric_logger \
    --ros-args \
    -p config_label:="$CONFIG_LABEL" \
    -p output_path:="$CSV_PATH" \
    -p use_sim_time:=True &
LOGGER_PID=$!
sleep 2

# -- 2) Testfall ausfuehren --
case "$1" in
    test_1)
        set_initial_pose
        start_experiment -7.4 6.4 -0.707 0.707
        ;;
    test_2)
        ros2 launch my_package szenario_launch.py &
        sleep 2
        ros2 topic pub --times 1 -w 0 /start_experiment std_msgs/msg/Empty "{}"
        ros2 topic pub --times 1 -w 0 /go_to_room std_msgs/msg/String "data: 'office_101'"
        ;;
     test_3)
        set_initial_pose
        start_experiment -26.0 -5.0 -0.707 0.707
        ;;
    *)
        echo "Usage: $0 {test_1|test_2|test_3} [config_label]"
        exit 1
        ;;
   
esac

# -- 3) Warten bis Logger fertig geschrieben hat --
echo "Warte auf Logger-Flush..."
sleep 5