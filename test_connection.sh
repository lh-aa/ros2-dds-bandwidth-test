#!/bin/bash

# 简单的 ROS2 DDS 连接测试脚本

echo "=== ROS2 DDS 连接测试 ==="

# 设置 DDS 实现
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp

# 启动订阅者
echo "启动订阅者..."
ros2 run dds_bandwidth_test bandwidth_subscriber --ros-args -p topic_name:=test_topic &
SUBSCRIBER_PID=$!

# 等待订阅者启动
sleep 3

# 检查话题是否存在
echo "检查话题..."
ros2 topic list | grep test_topic

# 启动发布者
echo "启动发布者..."
ros2 run dds_bandwidth_test bandwidth_publisher --ros-args \
    -p message_type:=string \
    -p message_size:=1024 \
    -p publish_rate:=5.0 \
    -p duration:=10.0 \
    -p topic_name:=test_topic

# 等待发布者完成
sleep 2

# 停止订阅者
echo "停止订阅者..."
kill $SUBSCRIBER_PID

echo "测试完成"

