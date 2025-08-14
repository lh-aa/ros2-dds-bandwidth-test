#!/bin/bash

# 极限带宽测试脚本 - 目标：100MB/s+

echo "=== 极限带宽测试 ==="
echo "目标：100MB/s+ (千兆网络理论带宽)"

# 设置 DDS 实现
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp

# 极限测试配置
MESSAGE_SIZE=16384  # 16MB
PUBLISH_RATE=500   # 500Hz (降低频率但增加消息大小)
DURATION=30        # 30秒
TOPIC_NAME="extreme_bandwidth_test"

echo "极限配置："
echo "  消息大小: ${MESSAGE_SIZE}KB"
echo "  发布频率: ${PUBLISH_RATE}Hz"
echo "  测试时长: ${DURATION}秒"
echo "  话题名称: ${TOPIC_NAME}"
echo "  理论带宽: $(echo "scale=2; ${MESSAGE_SIZE} * ${PUBLISH_RATE} / 1024" | bc) MB/s"

# 启动订阅者
echo "启动订阅者..."
ros2 run dds_bandwidth_test bandwidth_subscriber --ros-args \
    -p topic_name:=${TOPIC_NAME} \
    -p report_interval:=1.0 &
SUBSCRIBER_PID=$!

# 等待订阅者启动
sleep 3

# 检查话题
echo "检查话题..."
ros2 topic list | grep ${TOPIC_NAME}

# 启动发布者
echo "启动发布者..."
ros2 run dds_bandwidth_test bandwidth_publisher --ros-args \
    -p message_type:=string \
    -p message_size:=${MESSAGE_SIZE} \
    -p publish_rate:=${PUBLISH_RATE} \
    -p duration:=${DURATION} \
    -p topic_name:=${TOPIC_NAME}

# 等待发布者完成
sleep 5

# 停止订阅者
echo "停止订阅者..."
kill $SUBSCRIBER_PID

echo "极限测试完成！"

