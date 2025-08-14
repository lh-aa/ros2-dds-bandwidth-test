# ROS2 DDS Bandwidth Testing Suite

A comprehensive testing suite for ROS2 DDS (Data Distribution Service) bandwidth, latency, and performance analysis.

## 🚀 Features

- **Bandwidth Testing**: Measure message throughput and bandwidth usage
- **Latency Analysis**: Track message delivery latency and jitter  
- **Multiple Message Types**: Support for string, image, pose_array, pointcloud, laserscan, odometry, and twist messages
- **High-Speed Streaming**: Test real-time performance with configurable frequencies up to 1000+ Hz
- **Multi-Stream Testing**: Test concurrent data streams
- **Comprehensive Statistics**: Detailed performance metrics and reporting
- **DDS Optimization**: Pre-configured FastDDS and CycloneDDS settings for high performance

## 📦 Package Structure

```
ros2_dds_bandwidth_test/
├── dds_bandwidth_test/           # Main ROS2 package
│   ├── src/                      # Source code
│   │   ├── bandwidth_publisher.cpp
│   │   ├── bandwidth_subscriber.cpp
│   │   ├── simple_bandwidth_subscriber.cpp
│   │   └── high_speed_streamer.cpp
│   ├── launch/                   # Launch files
│   ├── config/                   # DDS configurations
│   ├── scripts/                  # Test automation
│   └── README.md                 # Detailed package documentation
├── QUICK_START.md                # Quick start guide
└── README.md                     # This file
```

## 🛠️ Installation

### Prerequisites
- ROS2 Humble or later
- C++ compiler (GCC 9+ or Clang 12+)
- Python 3.8+

### Build Instructions
```bash
# Clone the repository
git clone https://github.com/yourusername/ros2_dds_bandwidth_test.git
cd ros2_dds_bandwidth_test

# Build the package
colcon build --packages-select dds_bandwidth_test

# Source the workspace
source install/setup.bash
```

## 🎯 Quick Start

### Basic Bandwidth Test
```bash
cd dds_bandwidth_test
./scripts/run_bandwidth_tests.sh basic-test string 1024 100.0 30.0
```

### High-Speed Streaming Test
```bash
./scripts/run_bandwidth_tests.sh high-speed-test pointcloud 500.0 10000 30.0
```

### Run Test Suite
```bash
./scripts/run_bandwidth_tests.sh test-suite
```

## 📊 Test Scenarios

1. **Low-Bandwidth Testing**: String messages at low frequencies
2. **Medium-Bandwidth Testing**: Image messages at moderate rates
3. **High-Bandwidth Testing**: Large pose arrays at high frequencies
4. **Real-Time Performance**: High-frequency data streaming
5. **Multi-Stream Load Testing**: Concurrent data streams

## 🔧 DDS Configuration

The package includes optimized DDS configurations for both FastDDS and CycloneDDS:

- **FastDDS**: Optimized for high-throughput scenarios
- **CycloneDDS**: Optimized for low-latency scenarios

## 📈 Performance Metrics

- Message Rate (msg/s)
- Bandwidth (MB/s)
- Latency (min/max/average)
- Total Data Transferred
- Jitter Analysis

## 🤝 Contributing

1. Fork the repository
2. Create a feature branch
3. Make your changes
4. Add tests if applicable
5. Submit a pull request

## 📄 License

This project is licensed under the Apache License 2.0 - see the [LICENSE](LICENSE) file for details.

## 🙏 Acknowledgments

- ROS2 Community
- DDS Working Group
- Contributors and testers

## 📞 Support

If you encounter any issues or have questions:
- Open an issue on GitHub
- Check the documentation in `dds_bandwidth_test/README.md`
- Review the quick start guide in `QUICK_START.md`

---

**Happy Testing! 🚀** test ssh push
