#!/bin/bash

echo "=== 高带宽测试 ==="
echo "目标：100MB/s"

# 设置 DDS 实现
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp

# 高带宽配置
MESSAGE_SIZE=8192  # 8MB
PUBLISH_RATE=100   # 100Hz
DURATION=30        # 30秒
TOPIC_NAME="high_bandwidth_test"

echo "配置："
echo "  消息大小: ${MESSAGE_SIZE}KB"
echo "  发布频率: ${PUBLISH_RATE}Hz"
echo "  测试时长: ${DURATION}秒"
echo "  理论带宽: $(echo "scale=2; ${MESSAGE_SIZE} * ${PUBLISH_RATE} / 1024" | bc) MB/s"

# 启动订阅者
echo "启动订阅者..."
ros2 run dds_bandwidth_test bandwidth_subscriber --ros-args \
    -p topic_name:=${TOPIC_NAME} \
    -p report_interval:=1.0 &
SUBSCRIBER_PID=$!

# 等待订阅者启动
sleep 5

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

echo "测试完成"

