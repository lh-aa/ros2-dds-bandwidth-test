#!/bin/bash

echo "=== 简单连接测试 ==="

# 设置 DDS 实现
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp

# 启动订阅者
echo "启动订阅者..."
ros2 run dds_bandwidth_test bandwidth_subscriber --ros-args -p topic_name:=simple_test &
SUBSCRIBER_PID=$!

# 等待订阅者启动
echo "等待订阅者启动..."
sleep 5

# 检查话题
echo "检查话题..."
ros2 topic list | grep simple_test

# 启动发布者
echo "启动发布者..."
ros2 run dds_bandwidth_test bandwidth_publisher --ros-args \
    -p message_type:=string \
    -p message_size:=1024 \
    -p publish_rate:=10.0 \
    -p duration:=10.0 \
    -p topic_name:=simple_test

# 等待发布者完成
sleep 2

# 停止订阅者
echo "停止订阅者..."
kill $SUBSCRIBER_PID

echo "测试完成"
