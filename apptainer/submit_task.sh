#!/bin/bash
#$ -N fission_batch_job                   # 任务名称
#$ -cwd                                   # 在当前目录执行任务
#$ -o "logs/output.$JOB_ID.$TASK_ID.log"
#$ -e "logs/error.$JOB_ID.$TASK_ID.log"
#$ -t 1-10                               # 任务数组：共500个子任务
#$ -tc 10
#$ -j y                                   # 合并 stdout 和 stderr
#$ -pe smp 16                              # 每个任务占用核心数
#$ -l h_vmem=32G                           # 每个任务使用最多内存数
#$ -l h_rt=00:11:00                       # 每任务最大运行时间为11分钟

#  当前任务 ID（SGE 传入）
TASK_ID=${SGE_TASK_ID}

#  项目路径
BASE_DIR=$PWD
RESULT_DIR=$BASE_DIR/result
SCRIPT_PATH=$BASE_DIR/container_scrip.sh
SIF_PATH=$BASE_DIR/fission_fusion.sif

#  确保目录存在
mkdir -p $RESULT_DIR
mkdir -p logs

#  每个任务独立输出文件
RESULT_FILE=$RESULT_DIR/task_${TASK_ID}.csv

#  跳过已完成任务（支持断点续跑）
if [ -f "$RESULT_FILE" ]; then
  echo "[$(date)] Task $TASK_ID already done. Skipping."
  exit 0
fi

#  日志提示任务启动
echo "[$(date)] Launching task $TASK_ID → $RESULT_FILE"

#  启动容器任务
apptainer exec \
  --containall \
  --cleanenv \
  --bind $SCRIPT_PATH:/script.sh \
  --bind $RESULT_DIR:/results \
  $SIF_PATH \
  bash /script.sh /results/task_${TASK_ID}.csv

echo "[$(date)] Task $TASK_ID completed."
