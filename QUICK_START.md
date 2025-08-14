# ROS2 DDS Bandwidth Testing - Quick Start Guide

## 🚀 Quick Start

This package provides comprehensive tools for testing ROS2 DDS bandwidth, latency, and performance. It's located at `/home/aa/Music/tesst/ros2_dds_bandwidth_test`.

### Immediate Usage

1. **Build the package:**
   ```bash
   cd /home/aa/Music/tesst/ros2_dds_bandwidth_test
   colcon build --packages-select dds_bandwidth_test
   source install/setup.bash
   ```

2. **Run a quick bandwidth test:**
   ```bash
   cd dds_bandwidth_test
   ./scripts/run_bandwidth_tests.sh basic-test string 1024 100.0 30.0
   ```

3. **Run high-speed streaming test:**
   ```bash
   ./scripts/run_bandwidth_tests.sh high-speed-test pointcloud 500.0 10000 30.0
   ```

## 📊 Available Test Types

### Basic Bandwidth Tests
- **String messages**: Lightweight text data
- **Image messages**: Large binary data simulation
- **Pose array messages**: Structured geometric data

### High-Speed Streaming Tests
- **Point clouds**: 3D sensor data simulation
- **Laser scans**: 2D range sensor data
- **Odometry**: Robot position/velocity data
- **Twist**: Velocity commands

## 🛠️ Key Features

- **Configurable parameters**: Message size, frequency, duration
- **Real-time statistics**: Bandwidth, latency, message rate
- **Multiple DDS support**: FastDDS and CycloneDDS configurations
- **Multi-stream testing**: Concurrent data streams
- **Comprehensive reporting**: Detailed performance metrics

## 📁 Package Structure

```
dds_bandwidth_test/
├── src/                          # Source code
│   ├── bandwidth_publisher.cpp   # Message publisher
│   ├── bandwidth_subscriber.cpp  # Message subscriber
│   └── high_speed_streamer.cpp   # High-frequency streamer
├── launch/                       # Launch files
│   ├── bandwidth_test.launch.py  # Basic bandwidth tests
│   └── high_speed_test.launch.py # High-speed streaming
├── config/                       # DDS configurations
│   ├── fastdds_high_performance.xml
│   └── cyclonedds_high_performance.xml
├── scripts/                      # Test automation
│   └── run_bandwidth_tests.sh    # Main test script
└── README.md                     # Detailed documentation
```

## 🎯 Common Use Cases

### 1. Network Performance Testing
```bash
# Test network bandwidth with large messages
./scripts/run_bandwidth_tests.sh basic-test image 4096 50.0 60.0
```

### 2. Real-Time System Testing
```bash
# Test high-frequency data streaming
./scripts/run_bandwidth_tests.sh high-speed-test laserscan 1000.0
```

### 3. Multi-Node Testing
```bash
# Test multiple concurrent streams
./scripts/run_bandwidth_tests.sh high-speed-test pointcloud 200.0 10000 30.0 true
```

### 4. DDS Configuration Testing
```bash
# Setup FastDDS for high performance
./scripts/run_bandwidth_tests.sh setup-dds fastdds
./scripts/run_bandwidth_tests.sh basic-test string 2048 100.0 30.0
```

## 📈 Performance Metrics

The tools provide real-time metrics including:
- **Message Rate**: Messages per second
- **Bandwidth**: Megabytes per second
- **Latency**: Message delivery time (min/max/average)
- **Total Data**: Bytes transferred
- **Jitter**: Latency variation

## 🔧 DDS Configuration

For optimal performance, use the provided DDS configurations:

### FastDDS
```bash
export FASTRTPS_DEFAULT_PROFILES_FILE=/home/aa/Music/tesst/ros2_dds_bandwidth_test/dds_bandwidth_test/config/fastdds_high_performance.xml
```

### CycloneDDS
```bash
export CYCLONEDDS_URI=file:///home/aa/Music/tesst/ros2_dds_bandwidth_test/dds_bandwidth_test/config/cyclonedds_high_performance.xml
```

## 🚨 Troubleshooting

### Common Issues
1. **Build errors**: Install `catkin_pkg` with `pip install catkin_pkg`
2. **High latency**: Check network configuration and DDS settings
3. **Message loss**: Reduce publish rate or increase queue sizes
4. **Low bandwidth**: Verify message sizes and network capacity

### Performance Tips
- Use appropriate message types for your use case
- Configure DDS for your network environment
- Monitor system resources (CPU, memory, network)
- Test with realistic data sizes and rates

## 📚 Next Steps

1. Read the full `README.md` for detailed documentation
2. Explore the launch files for custom configurations
3. Modify the source code for specific testing needs
4. Create custom DDS configurations for your environment

## 🎉 Ready to Test!

Your ROS2 DDS bandwidth testing environment is ready. Start with a simple test and gradually increase complexity based on your needs.

```bash
# Start with this simple test
./scripts/run_bandwidth_tests.sh basic-test string 1024 100.0 30.0
```

Happy testing! 🚀 