#!/bin/bash
ros2 daemon stop
ros2 daemon start
# Define a function to clean up processes when the script is terminated
cleanup() {
    echo "Stopping ARGoS3, ROS 2, rosbag, and RViz..."
    pkill -f argos3 # Kill ARGoS3 process
    pkill -f ros2   # Kill all ROS 2 processes
    pkill -f rviz2  # Kill RViz process
    exit 0
}

# Trap SIGINT (Ctrl+C) and call cleanup function
trap cleanup SIGINT

source ~/fission_fusion_ws/install/setup.bash
# Export necessary environment variables
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/usr/local/lib/argos3:/home/tianfu/fission_fusion_ws/install/argos3_ros_bridge/lib
export ARGOS_PLUGIN_PATH=/home/tianfu/fission_fusion_ws/install/argos3_ros_bridge/lib

# Run ARGoS3 with the specified configuration file
argos3 -c fission_fusion/experiments/fission_fusion.argos results_file_path:=/home/tianfu/fission_fusion_ws/src/fission_fusion/data/2.csv &
ARGOS_PID=$! # Save the process ID (PID) of argos3

# Wait for ARGoS3 to initialize
sleep 3 # Increase sleep time if necessary

# Run the ROS 2 launch command in the background
ros2 launch fission_fusion run.launch.py &
ROS2_PID=$! # Save the process ID (PID) of ros2 launch

# Wait for ROS 2 nodes to initialize
sleep 3 # Increase sleep time if necessary

# Wait for 5 minutes (300 seconds) before stopping the current iteration
sleep 600

# Stop ARGoS3, ROS 2, rosbag, and RViz
echo "Stopping ARGoS3, ROS 2, rosbag, and RViz for iteration $i"
pkill -f argos3
pkill -f ros2
# pkill -f "ros2 bag"
pkill -f rviz2

# Wait for processes to terminate
wait $ROS2_PID
wait $ARGOS_PID

echo "completed"
