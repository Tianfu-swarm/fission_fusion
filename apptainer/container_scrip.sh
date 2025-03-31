#!/bin/bash
# 容器内执行的脚本

# 检查是否传入了结果文件路径参数
if [ -z "$1" ]; then
    echo "Error: No results file path provided"
    exit 1
fi

RESULTS_FILE="$1"

# ==== 设置 ROS_DOMAIN_ID（从外部传入）====
if [ -z "$ROS_DOMAIN_ID" ]; then
    echo "[ERROR] ROS_DOMAIN_ID not provided from host. Aborting!"
    exit 1
else
    echo "[INFO] Using ROS_DOMAIN_ID=$ROS_DOMAIN_ID"
    export ROS_DOMAIN_ID
fi

source /opt/container_env/fission_fusion_ws/install/setup.bash

# ROS 2 守护进程管理
ros2 daemon stop
ros2 daemon start

# 定义清理函数
cleanup() {
    echo "Stopping ARGoS3, ROS 2 and RViz..."
    pkill -f argos3
    pkill -f ros2
    pkill -f rviz2
    exit 0
}

# 捕获中断信号
trap cleanup SIGINT SIGTERM

# 初始化环境
export LD_LIBRARY_PATH=/usr/local/lib/argos3:/opt/ros/humble/lib:/opt/container_env/fission_fusion_ws/install/argos3_ros_bridge/lib:$LD_LIBRARY_PATH

export ARGOS_PLUGIN_PATH=/usr/local/lib/argos3:/opt/container_env/fission_fusion_ws/install/argos3_ros_bridge/lib

# Disable Fast DDS shared memory transport to avoid /dev/shm permission issue
# export RMW_FASTRTPS_USE_SHM=OFF

# 运行ARGoS3
echo "Starting ARGoS3 "
argos3 -c /opt/container_env/fission_fusion_ws/src/fission_fusion/experiments/fission_fusion.argos &
ARGOS_PID=$!

# 等待初始化
sleep 10

echo "[READY] ARGoS initialized, Now launching ROS 2 Controller"

# 运行ROS 2
echo "Starting ROS 2 nodes with output file: $RESULTS_FILE"
source /opt/container_env/fission_fusion_ws/install/setup.bash
ros2 launch fission_fusion run.launch.py use_rviz:=false results_file_path:="$RESULTS_FILE" &
ROS2_PID=$!

# 等待初始化
sleep 10

# 主运行周期 (10分钟)
echo "Running simulation for 600 seconds"
sleep 600

# 清理
echo "Stopping processes"
cleanup

echo "Simulation completed successfully"
exit 0