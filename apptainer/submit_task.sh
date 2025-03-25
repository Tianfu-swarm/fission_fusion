#!/bin/bash
#$ -N fission_batch_job                   
#$ -cwd                                   
#$ -o "logs/output.$JOB_ID.$TASK_ID.log"
#$ -e "logs/error.$JOB_ID.$TASK_ID.log"
#$ -t 1-1                               
#$ -tc 1
#$ -j y                                  
#$ -pe smp 16                             
#$ -l h_vmem=32G                          
#$ -l h_rt=00:11:00                       
#$ -q scc                                 # 明确使用 scc 队列的计算节点

TASK_ID=${SGE_TASK_ID}

echo "Running on node: $(hostname)"
echo "Date: $(date)"

echo "---- CPU and memory ----"
free -h
lscpu | grep "CPU(s):"

# 定义路径
BASE_DIR=$PWD
LOCAL_RESULT_DIR=/data/scc/$USER/result
FINAL_RESULT_DIR=$BASE_DIR/result
SCRIPT_PATH=$BASE_DIR/container_scrip.sh
SIF_PATH=$BASE_DIR/fission_fusion.sif
SHARED_LOCAL_SIF=/data/scc/$USER/fission_fusion.sif
LOCAL_SIF=/tmp/fission_fusion_task_${TASK_ID}.sif

# 创建目录
mkdir -p $LOCAL_RESULT_DIR
mkdir -p $FINAL_RESULT_DIR
mkdir -p logs

# 每个任务独立输出文件（本地存储）
LOCAL_RESULT_FILE=$LOCAL_RESULT_DIR/task_${TASK_ID}.csv
FINAL_RESULT_FILE=$FINAL_RESULT_DIR/task_${TASK_ID}.csv

# 跳过已完成任务（检查最终目录）
if [ -f "$FINAL_RESULT_FILE" ]; then
  echo "[$(date)] Task $TASK_ID already done. Skipping."
  exit 0
fi

# 如果共享容器镜像不存在，先从BASE_DIR复制一份
if [ ! -f "$SHARED_LOCAL_SIF" ]; then
    echo "[$(date)] Shared container not found. Copying to compute node..."
    cp $SIF_PATH $SHARED_LOCAL_SIF
fi

# 从共享镜像复制一份为本任务专属副本
cp $SHARED_LOCAL_SIF $LOCAL_SIF

# 日志提示任务启动
echo "[$(date)] Launching task $TASK_ID → $LOCAL_RESULT_FILE"

# 启动容器任务（数据写到本地存储）
apptainer exec \
  --containall \
  --cleanenv \
  --bind $SCRIPT_PATH:/script.sh \
  --bind $LOCAL_RESULT_DIR:/results \
  $LOCAL_SIF \
  bash /script.sh /results/task_${TASK_ID}.csv &

# 等待几秒，确保任务启动
sleep 10

# 等待容器任务结束
wait $CONTAINER_PID

echo "[$(date)] Task $TASK_ID completed."

# 容器结束后，将数据复制回登录节点（最终网络存储）
echo "[$(date)] Copying results back to login node storage..."
rsync -av $LOCAL_RESULT_FILE $FINAL_RESULT_FILE

# 清理仅本任务的容器副本
rm -f $LOCAL_SIF
rm -f $LOCAL_RESULT_FILE

echo "[$(date)] Cleanup done for Task $TASK_ID."
