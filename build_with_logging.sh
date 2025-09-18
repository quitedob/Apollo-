#!/bin/bash
# Apollo 编译日志记录脚本
# 用于跟踪编译过程中的警告和错误

set -e

LOG_FILE="/apollo_workspace/build_log_$(date +%Y%m%d_%H%M%S).txt"
WARNING_COUNT=0
ERROR_COUNT=0

echo "开始编译 Apollo 项目..." | tee "$LOG_FILE"
echo "日志文件: $LOG_FILE" | tee -a "$LOG_FILE"
echo "开始时间: $(date)" | tee -a "$LOG_FILE"
echo "==========================================" | tee -a "$LOG_FILE"

# 执行编译命令并捕获输出
bazel build //modules/planning/... 2>&1 | tee -a "$LOG_FILE" | while read -r line; do
    # 统计警告数量
    if echo "$line" | grep -q "warning:"; then
        ((WARNING_COUNT++))
    fi

    # 统计错误数量
    if echo "$line" | grep -q "error:"; then
        ((ERROR_COUNT++))
    fi

    # 实时显示重要的警告信息
    if echo "$line" | grep -q "warning: comparison of integer expressions of different signedness"; then
        echo "[签名性警告] $line" >&2
    fi
done

echo "==========================================" | tee -a "$LOG_FILE"
echo "编译完成时间: $(date)" | tee -a "$LOG_FILE"
echo "警告总数: $WARNING_COUNT" | tee -a "$LOG_FILE"
echo "错误总数: $ERROR_COUNT" | tee -a "$LOG_FILE"
echo "==========================================" | tee -a "$LOG_FILE"

# 显示签名性警告的汇总
echo "签名性警告汇总:" | tee -a "$LOG_FILE"
grep -n "warning: comparison of integer expressions of different signedness" "$LOG_FILE" | head -10 | tee -a "$LOG_FILE"

if [ $WARNING_COUNT -gt 0 ]; then
    echo "警告: 发现 $WARNING_COUNT 个编译警告，请检查日志文件: $LOG_FILE" >&2
fi

if [ $ERROR_COUNT -gt 0 ]; then
    echo "错误: 发现 $ERROR_COUNT 个编译错误，请检查日志文件: $LOG_FILE" >&2
    exit 1
fi

echo "编译完成！" | tee -a "$LOG_FILE"
