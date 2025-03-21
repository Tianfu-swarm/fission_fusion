#!/bin/bash
    ros2 daemon stop
    ros2 daemon start
# Define a function to clean up processes when the script is terminated
cleanup() {
    echo "Stopping ARGoS3, ROS 2, rosbag, and RViz..."
    pkill -f argos3  # Kill ARGoS3 process
    pkill -f ros2    # Kill all ROS 2 processes
    pkill -f rviz2   # Kill RViz process
    exit 0
}

# Trap SIGINT (Ctrl+C) and call cleanup function
trap cleanup SIGINT

# Loop to modify the config file and run the experiment
for i in {1..10}; do  # Change 10 to the number of iterations you want
    echo "Starting iteration $i"

    # Srestart ros2 daemon
    ros2 daemon stop
    ros2 daemon start
    sleep 3

     # 清理共享内存
    echo "Cleaning up Fast RTPS shared memory..."
    # 清理 Fast DDS 共享内存文件
    rm -rf /dev/shm/fastrtps_*
    rm -rf /dev/shm/foonet_*
    
    # 确保端口被释放
    echo "Checking and killing processes on blocked Fast RTPS ports..."
    for PORT in 7447 7448 7449 7489 7495; do
        PID=$(lsof -t -i :$PORT)
        if [ -n "$PID" ]; then
            echo "Port $PORT is occupied by process $PID, killing..."
            kill -9 $PID
        fi
    done

    for FILE in /dev/shm/fastrtps_* /dev/shm/foonet_*; do
    PID=$(fuser $FILE 2>/dev/null)
    if [ -n "$PID" ]; then
        echo "Shared memory file $FILE is used by process $PID, killing..."
        kill -9 $PID
    fi
    done



    source ~/fission_fusion_ws/install/setup.bash  
    # Export necessary environment variables
    export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/usr/local/lib/argos3:/opt/container_env/fission_fusion_ws/src/fission_fusion/lib
    export ARGOS_PLUGIN_PATH=/opt/container_env/fission_fusion_ws/src/fission_fusion/lib

    # Modify the results_file_name in the config file
    sed -i "s/results_file_name: .*/results_file_name: \"$i.csv\"/" /home/tianfu/fission_fusion_ws/install/fission_fusion/share/fission_fusion/config/config.yaml

    # Run ARGoS3 with the specified configuration file
    argos3 -c fission_fusion/experiments/fission_fusion.argos &
    ARGOS_PID=$!  # Save the process ID (PID) of argos3

    # Wait for ARGoS3 to initialize
    sleep 3  # Increase sleep time if necessary

    # Run the ROS 2 launch command in the background
    ros2 launch fission_fusion run.launch.py &
    ROS2_PID=$!  # Save the process ID (PID) of ros2 launch

    # Wait for ROS 2 nodes to initialize
    sleep 3  # Increase sleep time if necessary

    # Wait for 5 minutes (300 seconds) before stopping the current iteration
    sleep 10

    # Stop ARGoS3, ROS 2, rosbag, and RViz
    echo "Stopping ARGoS3, ROS 2, rosbag, and RViz for iteration $i"
    pkill -f argos3
    pkill -f ros2
    # pkill -f "ros2 bag"
    pkill -f rviz2

    # Wait for processes to terminate
    wait $ROS2_PID
    wait $ARGOS_PID

    echo "Iteration $i completed"
done

echo "All iterations completed"