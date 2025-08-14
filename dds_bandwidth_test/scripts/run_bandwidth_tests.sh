#!/bin/bash

# ROS2 DDS Bandwidth Testing Script
# This script provides easy-to-use commands for running different bandwidth tests

set -e

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# Function to print colored output
print_info() {
    echo -e "${BLUE}[INFO]${NC} $1"
}

print_success() {
    echo -e "${GREEN}[SUCCESS]${NC} $1"
}

print_warning() {
    echo -e "${YELLOW}[WARNING]${NC} $1"
}

print_error() {
    echo -e "${RED}[ERROR]${NC} $1"
}

# Function to check if ROS2 is available
check_ros2() {
    if ! command -v ros2 &> /dev/null; then
        print_error "ROS2 is not installed or not in PATH"
        exit 1
    fi
    print_success "ROS2 found: $(ros2 --version 2>&1 | head -n1)"
}

# Function to build the package
build_package() {
    print_info "Building dds_bandwidth_test package..."
    cd /home/aa/Music/tesst/ros2_dds_bandwidth_test
    colcon build --packages-select dds_bandwidth_test
    source install/setup.bash
    print_success "Package built successfully"
}

# Function to set DDS configuration
setup_dds() {
    local dds_type=$1
    local config_dir="/home/aa/Music/tesst/ros2_dds_bandwidth_test/dds_bandwidth_test/config"
    
    case $dds_type in
        "fastdds")
            export FASTRTPS_DEFAULT_PROFILES_FILE="$config_dir/fastdds_high_performance.xml"
            print_info "Using FastDDS configuration: $FASTRTPS_DEFAULT_PROFILES_FILE"
            ;;
        "cyclonedds")
            export CYCLONEDDS_URI="file://$config_dir/cyclonedds_high_performance.xml"
            print_info "Using CycloneDDS configuration: $CYCLONEDDS_URI"
            ;;
        *)
            print_warning "Unknown DDS type: $dds_type. Using default configuration."
            ;;
    esac
}

# Function to run basic bandwidth test
run_basic_test() {
    local message_type=${1:-"string"}
    local message_size=${2:-"1024"}
    local publish_rate=${3:-"100.0"}
    local duration=${4:-"30.0"}
    
    print_info "Running basic bandwidth test:"
    print_info "  Message type: $message_type"
    print_info "  Message size: $message_size KB"
    print_info "  Publish rate: $publish_rate Hz"
    print_info "  Duration: $duration seconds"
    
    ros2 launch dds_bandwidth_test bandwidth_test.launch.py \
        message_type:=$message_type \
        message_size:=$message_size \
        publish_rate:=$publish_rate \
        duration:=$duration
}

# Function to run high-speed streaming test
run_high_speed_test() {
    local stream_type=${1:-"pointcloud"}
    local frequency=${2:-"500.0"}
    local point_count=${3:-"10000"}
    local duration=${4:-"30.0"}
    local multiple_streams=${5:-"false"}
    
    print_info "Running high-speed streaming test:"
    print_info "  Stream type: $stream_type"
    print_info "  Frequency: $frequency Hz"
    print_info "  Point count: $point_count"
    print_info "  Duration: $duration seconds"
    print_info "  Multiple streams: $multiple_streams"
    
    ros2 launch dds_bandwidth_test high_speed_test.launch.py \
        stream_type:=$stream_type \
        frequency:=$frequency \
        point_count:=$point_count \
        duration:=$duration \
        enable_multiple_streams:=$multiple_streams
}

# Function to run comprehensive test suite
run_test_suite() {
    print_info "Running comprehensive bandwidth test suite..."
    
    # Test 1: Low bandwidth string messages
    print_info "Test 1: Low bandwidth string messages"
    run_basic_test "string" "100" "10.0" "10.0"
    sleep 2
    
    # Test 2: Medium bandwidth image messages
    print_info "Test 2: Medium bandwidth image messages"
    run_basic_test "image" "1024" "30.0" "10.0"
    sleep 2
    
    # Test 3: High bandwidth pose array messages
    print_info "Test 3: High bandwidth pose array messages"
    run_basic_test "pose_array" "4096" "100.0" "10.0"
    sleep 2
    
    # Test 4: High-speed point cloud streaming
    print_info "Test 4: High-speed point cloud streaming"
    run_high_speed_test "pointcloud" "500.0" "5000" "10.0" "false"
    sleep 2
    
    # Test 5: Multiple concurrent streams
    print_info "Test 5: Multiple concurrent streams"
    run_high_speed_test "pointcloud" "200.0" "10000" "10.0" "true"
    
    print_success "Test suite completed"
}

# Function to show usage
show_usage() {
    echo "ROS2 DDS Bandwidth Testing Script"
    echo ""
    echo "Usage: $0 [COMMAND] [OPTIONS]"
    echo ""
    echo "Commands:"
    echo "  build                    Build the dds_bandwidth_test package"
    echo "  basic-test [TYPE] [SIZE] [RATE] [DURATION]"
    echo "                           Run basic bandwidth test"
    echo "                           TYPE: string, image, pose_array (default: string)"
    echo "                           SIZE: message size in KB (default: 1024)"
    echo "                           RATE: publish rate in Hz (default: 100.0)"
    echo "                           DURATION: test duration in seconds (default: 30.0)"
    echo "  high-speed-test [TYPE] [FREQ] [POINTS] [DURATION] [MULTI]"
    echo "                           Run high-speed streaming test"
    echo "                           TYPE: pointcloud, laserscan, odometry, twist (default: pointcloud)"
    echo "                           FREQ: frequency in Hz (default: 500.0)"
    echo "                           POINTS: point count (default: 10000)"
    echo "                           DURATION: test duration in seconds (default: 30.0)"
    echo "                           MULTI: enable multiple streams (default: false)"
    echo "  test-suite               Run comprehensive test suite"
    echo "  setup-dds [TYPE]         Setup DDS configuration"
    echo "                           TYPE: fastdds, cyclonedds"
    echo "  help                     Show this help message"
    echo ""
    echo "Examples:"
    echo "  $0 build"
    echo "  $0 setup-dds fastdds"
    echo "  $0 basic-test string 2048 50.0 60.0"
    echo "  $0 high-speed-test pointcloud 1000.0 20000 30.0 true"
    echo "  $0 test-suite"
}

# Main script logic
main() {
    check_ros2
    
    case $1 in
        "build")
            build_package
            ;;
        "basic-test")
            run_basic_test $2 $3 $4 $5
            ;;
        "high-speed-test")
            run_high_speed_test $2 $3 $4 $5 $6
            ;;
        "test-suite")
            run_test_suite
            ;;
        "setup-dds")
            setup_dds $2
            ;;
        "help"|"--help"|"-h"|"")
            show_usage
            ;;
        *)
            print_error "Unknown command: $1"
            show_usage
            exit 1
            ;;
    esac
}

# Run main function with all arguments
main "$@" 