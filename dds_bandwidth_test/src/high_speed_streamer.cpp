#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <std_msgs/msg/header.hpp>
#include <chrono>
#include <memory>
#include <random>
#include <vector>

class HighSpeedStreamer : public rclcpp::Node
{
public:
  HighSpeedStreamer() : Node("high_speed_streamer")
  {
    // Parameters
    this->declare_parameter("stream_type", "pointcloud");
    this->declare_parameter("frequency", 1000.0);
    this->declare_parameter("point_count", 10000);
    this->declare_parameter("duration", 30.0);
    this->declare_parameter("enable_multiple_streams", false);

    stream_type_ = this->get_parameter("stream_type").as_string();
    frequency_ = this->get_parameter("frequency").as_double();
    point_count_ = this->get_parameter("point_count").as_int();
    duration_ = this->get_parameter("duration").as_double();
    enable_multiple_streams_ = this->get_parameter("enable_multiple_streams").as_bool();

    // Initialize random number generator
    rng_.seed(std::chrono::steady_clock::now().time_since_epoch().count());
    dist_ = std::uniform_real_distribution<float>(-10.0f, 10.0f);

    // Create publishers based on stream type
    if (stream_type_ == "pointcloud") {
      pointcloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
        "high_speed_pointcloud", 10);
    } else if (stream_type_ == "laserscan") {
      laserscan_pub_ = this->create_publisher<sensor_msgs::msg::LaserScan>(
        "high_speed_laserscan", 10);
    } else if (stream_type_ == "odometry") {
      odometry_pub_ = this->create_publisher<nav_msgs::msg::Odometry>(
        "high_speed_odometry", 10);
    } else if (stream_type_ == "twist") {
      twist_pub_ = this->create_publisher<geometry_msgs::msg::TwistStamped>(
        "high_speed_twist", 10);
    }

    // Create multiple stream publishers if enabled
    if (enable_multiple_streams_) {
      for (int i = 0; i < 5; ++i) {
        auto pub = this->create_publisher<sensor_msgs::msg::PointCloud2>(
          "stream_" + std::to_string(i), 10);
        multi_stream_pubs_.push_back(pub);
      }
    }

    // Create timer
    timer_ = this->create_wall_timer(
      std::chrono::duration<double>(1.0 / frequency_),
      std::bind(&HighSpeedStreamer::timer_callback, this));

    // Statistics
    start_time_ = this->now();
    message_count_ = 0;
    total_bytes_ = 0;

    RCLCPP_INFO(this->get_logger(), 
      "Starting high-speed streamer: %s, %.1f Hz, %d points, %.1f seconds",
      stream_type_.c_str(), frequency_, point_count_, duration_);
  }

private:
  void timer_callback()
  {
    auto current_time = this->now();
    auto elapsed = (current_time - start_time_).seconds();

    if (elapsed >= duration_) {
      // Print final statistics
      double avg_rate = message_count_ / elapsed;
      double avg_bandwidth = total_bytes_ / elapsed;
      
      RCLCPP_INFO(this->get_logger(), 
        "Stream completed: %ld messages, %.2f msg/s, %.2f MB/s",
        message_count_, avg_rate, avg_bandwidth / (1024 * 1024));
      
      rclcpp::shutdown();
      return;
    }

    message_count_++;
    
    if (stream_type_ == "pointcloud") {
      publish_pointcloud();
    } else if (stream_type_ == "laserscan") {
      publish_laserscan();
    } else if (stream_type_ == "odometry") {
      publish_odometry();
    } else if (stream_type_ == "twist") {
      publish_twist();
    }

    // Publish to multiple streams if enabled
    if (enable_multiple_streams_) {
      for (auto& pub : multi_stream_pubs_) {
        auto pc = create_pointcloud();
        pub->publish(pc);
        total_bytes_ += pc.data.size();
      }
    }

    // Print progress every 1000 messages
    if (message_count_ % 1000 == 0) {
      double current_rate = message_count_ / elapsed;
      double current_bandwidth = total_bytes_ / elapsed;
      RCLCPP_INFO(this->get_logger(), 
        "Progress: %ld messages, %.2f msg/s, %.2f MB/s",
        message_count_, current_rate, current_bandwidth / (1024 * 1024));
    }
  }

  void publish_pointcloud()
  {
    auto pc = create_pointcloud();
    pointcloud_pub_->publish(pc);
    total_bytes_ += pc.data.size();
  }

  void publish_laserscan()
  {
    auto scan = sensor_msgs::msg::LaserScan();
    scan.header.stamp = this->now();
    scan.header.frame_id = "laser_frame";
    scan.angle_min = -M_PI;
    scan.angle_max = M_PI;
    scan.angle_increment = 2.0 * M_PI / 360.0;
    scan.time_increment = 0.0;
    scan.scan_time = 1.0 / frequency_;
    scan.range_min = 0.1;
    scan.range_max = 100.0;
    
    scan.ranges.resize(360);
    for (auto& range : scan.ranges) {
      range = 5.0 + dist_(rng_) * 0.1; // Random range around 5m
    }
    
    laserscan_pub_->publish(scan);
    total_bytes_ += scan.ranges.size() * sizeof(float);
  }

  void publish_odometry()
  {
    auto odom = nav_msgs::msg::Odometry();
    odom.header.stamp = this->now();
    odom.header.frame_id = "odom";
    odom.child_frame_id = "base_link";
    
    // Set position
    odom.pose.pose.position.x = dist_(rng_);
    odom.pose.pose.position.y = dist_(rng_);
    odom.pose.pose.position.z = 0.0;
    odom.pose.pose.orientation.w = 1.0;
    odom.pose.pose.orientation.x = 0.0;
    odom.pose.pose.orientation.y = 0.0;
    odom.pose.pose.orientation.z = 0.0;
    
    // Set velocity
    odom.twist.twist.linear.x = dist_(rng_) * 0.1;
    odom.twist.twist.linear.y = dist_(rng_) * 0.1;
    odom.twist.twist.linear.z = 0.0;
    odom.twist.twist.angular.x = 0.0;
    odom.twist.twist.angular.y = 0.0;
    odom.twist.twist.angular.z = dist_(rng_) * 0.1;
    
    odometry_pub_->publish(odom);
    total_bytes_ += sizeof(nav_msgs::msg::Odometry);
  }

  void publish_twist()
  {
    auto twist = geometry_msgs::msg::TwistStamped();
    twist.header.stamp = this->now();
    twist.header.frame_id = "base_link";
    
    twist.twist.linear.x = dist_(rng_) * 0.1;
    twist.twist.linear.y = dist_(rng_) * 0.1;
    twist.twist.linear.z = 0.0;
    twist.twist.angular.x = 0.0;
    twist.twist.angular.y = 0.0;
    twist.twist.angular.z = dist_(rng_) * 0.1;
    
    twist_pub_->publish(twist);
    total_bytes_ += sizeof(geometry_msgs::msg::TwistStamped);
  }

  sensor_msgs::msg::PointCloud2 create_pointcloud()
  {
    auto pc = sensor_msgs::msg::PointCloud2();
    pc.header.stamp = this->now();
    pc.header.frame_id = "map";
    
    // Set point cloud fields
    pc.fields.resize(3);
    pc.fields[0].name = "x";
    pc.fields[0].offset = 0;
    pc.fields[0].datatype = sensor_msgs::msg::PointField::FLOAT32;
    pc.fields[0].count = 1;
    
    pc.fields[1].name = "y";
    pc.fields[1].offset = 4;
    pc.fields[1].datatype = sensor_msgs::msg::PointField::FLOAT32;
    pc.fields[1].count = 1;
    
    pc.fields[2].name = "z";
    pc.fields[2].offset = 8;
    pc.fields[2].datatype = sensor_msgs::msg::PointField::FLOAT32;
    pc.fields[2].count = 1;
    
    pc.point_step = 12; // 3 floats * 4 bytes
    pc.row_step = pc.point_step * point_count_;
    pc.height = 1;
    pc.width = point_count_;
    pc.is_dense = true;
    
    // Generate random points
    pc.data.resize(point_count_ * pc.point_step);
    for (int i = 0; i < point_count_; ++i) {
      float x = dist_(rng_);
      float y = dist_(rng_);
      float z = dist_(rng_) * 0.1; // Small z variation
      
      memcpy(&pc.data[i * pc.point_step], &x, sizeof(float));
      memcpy(&pc.data[i * pc.point_step + 4], &y, sizeof(float));
      memcpy(&pc.data[i * pc.point_step + 8], &z, sizeof(float));
    }
    
    return pc;
  }

  // Publishers
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_pub_;
  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr laserscan_pub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odometry_pub_;
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr twist_pub_;
  std::vector<rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr> multi_stream_pubs_;
  
  // Timer
  rclcpp::TimerBase::SharedPtr timer_;
  
  // Parameters
  std::string stream_type_;
  double frequency_;
  int point_count_;
  double duration_;
  bool enable_multiple_streams_;
  
  // Statistics
  rclcpp::Time start_time_;
  size_t message_count_;
  size_t total_bytes_;
  
  // Random number generation
  std::mt19937 rng_;
  std::uniform_real_distribution<float> dist_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<HighSpeedStreamer>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
} 