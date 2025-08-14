# ROS2 DDS Bandwidth Testing Tools

This package provides comprehensive tools for testing ROS2 DDS (Data Distribution Service) bandwidth, latency, and performance under various conditions. It includes multiple testing scenarios for different message types and high-speed streaming applications.

## Features

- **Bandwidth Testing**: Measure message throughput and bandwidth usage
- **Latency Analysis**: Track message delivery latency and jitter
- **Multiple Message Types**: Support for string, image, pose_array, pointcloud, laserscan, odometry, and twist messages
- **High-Speed Streaming**: Test real-time performance with configurable frequencies up to 1000+ Hz
- **Multi-Stream Testing**: Test concurrent data streams
- **Comprehensive Statistics**: Detailed performance metrics and reporting

## Components

### 1. Bandwidth Publisher (`bandwidth_publisher`)
Generates test messages at configurable rates and sizes.

**Parameters:**
- `message_type`: Type of message (string, image, pose_array)
- `message_size`: Size in KB
- `publish_rate`: Publishing frequency in Hz
- `duration`: Test duration in seconds
- `topic_name`: Topic name for publishing

### 2. Bandwidth Subscriber (`bandwidth_subscriber`)
Receives messages and calculates performance metrics.

**Parameters:**
- `topic_name`: Topic name to subscribe to
- `window_size`: Size of rolling statistics window
- `report_interval`: Interval for statistics reporting in seconds

### 3. High-Speed Streamer (`high_speed_streamer`)
Generates high-frequency data streams for real-time testing.

**Parameters:**
- `stream_type`: Type of stream (pointcloud, laserscan, odometry, twist)
- `frequency`: Streaming frequency in Hz
- `point_count`: Number of points in pointcloud
- `duration`: Test duration in seconds
- `enable_multiple_streams`: Enable multiple concurrent streams

## Installation and Build

```bash
# Navigate to the workspace
cd /home/aa/Music/tesst/ros2_dds_bandwidth_test

# Build the package
colcon build --packages-select dds_bandwidth_test

# Source the workspace
source install/setup.bash
```

## Usage Examples

### Basic Bandwidth Test

Test string messages at 100 Hz for 60 seconds:

```bash
# Terminal 1: Start the test
ros2 launch dds_bandwidth_test bandwidth_test.launch.py \
  message_type:=string \
  message_size:=1024 \
  publish_rate:=100.0 \
  duration:=60.0
```

### High-Speed Point Cloud Test

Test point cloud streaming at 500 Hz:

```bash
# Terminal 1: Start high-speed streaming
ros2 launch dds_bandwidth_test high_speed_test.launch.py \
  stream_type:=pointcloud \
  frequency:=500.0 \
  point_count:=5000 \
  duration:=30.0
```

### Multiple Concurrent Streams

Test multiple point cloud streams simultaneously:

```bash
# Terminal 1: Start multiple streams
ros2 launch dds_bandwidth_test high_speed_test.launch.py \
  stream_type:=pointcloud \
  frequency:=200.0 \
  point_count:=10000 \
  enable_multiple_streams:=true
```

### Manual Node Execution

You can also run nodes individually:

```bash
# Terminal 1: Publisher
ros2 run dds_bandwidth_test bandwidth_publisher \
  --ros-args \
  -p message_type:=image \
  -p message_size:=2048 \
  -p publish_rate:=50.0

# Terminal 2: Subscriber
ros2 run dds_bandwidth_test bandwidth_subscriber \
  --ros-args \
  -p topic_name:=bandwidth_test \
  -p report_interval:=2.0
```

## Test Scenarios

### 1. Low-Bandwidth Testing
```bash
ros2 launch dds_bandwidth_test bandwidth_test.launch.py \
  message_type:=string \
  message_size:=100 \
  publish_rate:=10.0
```

### 2. Medium-Bandwidth Testing
```bash
ros2 launch dds_bandwidth_test bandwidth_test.launch.py \
  message_type:=image \
  message_size:=1024 \
  publish_rate:=30.0
```

### 3. High-Bandwidth Testing
```bash
ros2 launch dds_bandwidth_test bandwidth_test.launch.py \
  message_type:=pose_array \
  message_size:=4096 \
  publish_rate:=100.0
```

### 4. Real-Time Performance Testing
```bash
ros2 launch dds_bandwidth_test high_speed_test.launch.py \
  stream_type:=laserscan \
  frequency:=1000.0
```

### 5. Multi-Stream Load Testing
```bash
ros2 launch dds_bandwidth_test high_speed_test.launch.py \
  stream_type:=pointcloud \
  frequency:=200.0 \
  enable_multiple_streams:=true
```

## Performance Metrics

The tools provide comprehensive performance metrics:

- **Message Rate**: Messages per second
- **Bandwidth**: Megabytes per second
- **Latency**: Message delivery time (min/max/average)
- **Total Data**: Total bytes transferred
- **Jitter**: Latency variation

## DDS Configuration

For optimal performance, consider configuring DDS settings:

### FastDDS Configuration
Create a `fastdds.xml` file:
```xml
<?xml version="1.0" encoding="UTF-8" ?>
<dds>
    <profiles xmlns="http://www.eprosima.com/XMLSchemas/fastRTPS_Profiles">
        <transport_descriptors>
            <transport_descriptor>
                <transport_id>udp_transport</transport_id>
                <type>UDPv4</type>
                <maxMessageSize>65500</maxMessageSize>
                <sendBufferSize>65536</sendBufferSize>
                <receiveBufferSize>65536</receiveBufferSize>
            </transport_descriptor>
        </transport_descriptors>
        
        <participant profile_name="high_performance_profile">
            <rtps>
                <userTransports>
                    <transport_id>udp_transport</transport_id>
                </userTransports>
                <useBuiltinTransports>false</useBuiltinTransports>
            </rtps>
        </participant>
    </profiles>
</dds>
```

Set environment variable:
```bash
export FASTRTPS_DEFAULT_PROFILES_FILE=/path/to/fastdds.xml
```

### CycloneDDS Configuration
Create a `cyclonedds.xml` file:
```xml
<?xml version="1.0" encoding="UTF-8" ?>
<CycloneDDS>
    <Domain>
        <General>
            <AllowMulticast>true</AllowMulticast>
            <MaxMessageSize>65500B</MaxMessageSize>
        </General>
        <Internal>
            <Watermarks>
                <WhcHigh>500kB</WhcHigh>
            </Watermarks>
        </Internal>
    </Domain>
</CycloneDDS>
```

Set environment variable:
```bash
export CYCLONEDDS_URI=file:///path/to/cyclonedds.xml
```

## Troubleshooting

### Common Issues

1. **High Latency**: Check network configuration and DDS settings
2. **Message Loss**: Increase queue sizes or reduce publish rate
3. **Low Bandwidth**: Verify message sizes and network capacity
4. **Build Errors**: Ensure all dependencies are installed

### Performance Tips

1. Use appropriate message types for your use case
2. Configure DDS for your network environment
3. Monitor system resources (CPU, memory, network)
4. Test with realistic data sizes and rates
5. Consider using QoS settings for critical applications

## Contributing

To add new message types or testing scenarios:

1. Add new message type handling in the nodes
2. Update CMakeLists.txt with new dependencies
3. Create corresponding launch files
4. Update this README with usage examples

## License

This project is licensed under the Apache License 2.0. 