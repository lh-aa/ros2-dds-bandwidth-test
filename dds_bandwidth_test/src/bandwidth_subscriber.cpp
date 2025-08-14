#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <chrono>
#include <memory>
#include <string>
#include <vector>
#include <deque>
#include <numeric>

class BandwidthSubscriber : public rclcpp::Node
{
public:
  BandwidthSubscriber() : Node("bandwidth_subscriber")
  {
    // Parameters
    this->declare_parameter("topic_name", "bandwidth_test");
    this->declare_parameter("window_size", 1000);
    this->declare_parameter("report_interval", 5.0);

    topic_name_ = this->get_parameter("topic_name").as_string();
    window_size_ = this->get_parameter("window_size").as_int();
    report_interval_ = this->get_parameter("report_interval").as_double();

    // Create subscribers for all message types with unique topic names
    string_sub_ = this->create_subscription<std_msgs::msg::String>(
      topic_name_ + "_string", 10,
      std::bind(&BandwidthSubscriber::string_callback, this, std::placeholders::_1));

    image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
      topic_name_ + "_image", 10,
      std::bind(&BandwidthSubscriber::image_callback, this, std::placeholders::_1));

    pose_array_sub_ = this->create_subscription<geometry_msgs::msg::PoseArray>(
      topic_name_ + "_pose_array", 10,
      std::bind(&BandwidthSubscriber::pose_array_callback, this, std::placeholders::_1));

    // Create timer for periodic reporting
    report_timer_ = this->create_wall_timer(
      std::chrono::duration<double>(report_interval_),
      std::bind(&BandwidthSubscriber::report_statistics, this));

    // Statistics
    start_time_ = this->now();
    message_count_ = 0;
    total_bytes_ = 0;
    last_message_time_ = this->now();

    RCLCPP_INFO(this->get_logger(), 
      "Starting bandwidth subscriber on topic: %s", topic_name_.c_str());
  }

private:
  void string_callback(const std_msgs::msg::String::SharedPtr msg)
  {
    process_message(msg->data.size());
  }

  void image_callback(const sensor_msgs::msg::Image::SharedPtr msg)
  {
    process_message(msg->data.size());
  }

  void pose_array_callback(const geometry_msgs::msg::PoseArray::SharedPtr msg)
  {
    // Calculate approximate size of pose array
    size_t size = sizeof(geometry_msgs::msg::PoseArray);
    size += msg->poses.size() * sizeof(geometry_msgs::msg::Pose);
    process_message(size);
  }

  void process_message(size_t message_size)
  {
    auto current_time = this->now();
    
    // Calculate latency if we have a timestamp
    if (message_count_ > 0) {
      double latency = (current_time - last_message_time_).seconds() * 1000.0; // Convert to ms
      latencies_.push_back(latency);
      
      // Keep only the last window_size_ latencies
      if (latencies_.size() > static_cast<size_t>(window_size_)) {
        latencies_.pop_front();
      }
    }

    message_count_++;
    total_bytes_ += message_size;
    last_message_time_ = current_time;

    // Store message rate samples
    auto elapsed = (current_time - start_time_).seconds();
    if (elapsed > 0) {
      double current_rate = message_count_ / elapsed;
      message_rates_.push_back(current_rate);
      
      if (message_rates_.size() > static_cast<size_t>(window_size_)) {
        message_rates_.pop_front();
      }
    }
  }

  void report_statistics()
  {
    auto current_time = this->now();
    auto elapsed = (current_time - start_time_).seconds();
    
    if (elapsed <= 0) return;

    double avg_rate = message_count_ / elapsed;
    double avg_bandwidth = total_bytes_ / elapsed;
    
    // Calculate latency statistics
    double avg_latency = 0.0;
    double min_latency = std::numeric_limits<double>::max();
    double max_latency = 0.0;
    
    if (!latencies_.empty()) {
      avg_latency = std::accumulate(latencies_.begin(), latencies_.end(), 0.0) / latencies_.size();
      min_latency = *std::min_element(latencies_.begin(), latencies_.end());
      max_latency = *std::max_element(latencies_.begin(), latencies_.end());
    }

    // Calculate rate statistics
    double avg_message_rate = 0.0;
    double min_message_rate = std::numeric_limits<double>::max();
    double max_message_rate = 0.0;
    
    if (!message_rates_.empty()) {
      avg_message_rate = std::accumulate(message_rates_.begin(), message_rates_.end(), 0.0) / message_rates_.size();
      min_message_rate = *std::min_element(message_rates_.begin(), message_rates_.end());
      max_message_rate = *std::max_element(message_rates_.begin(), message_rates_.end());
    }

    RCLCPP_INFO(this->get_logger(), 
      "=== Bandwidth Statistics (%.1f s elapsed) ===", elapsed);
    RCLCPP_INFO(this->get_logger(), 
      "Messages: %ld, Rate: %.2f msg/s (min: %.2f, max: %.2f, avg: %.2f)",
      message_count_, avg_rate, min_message_rate, max_message_rate, avg_message_rate);
    RCLCPP_INFO(this->get_logger(), 
      "Bandwidth: %.2f MB/s", avg_bandwidth / (1024 * 1024));
    RCLCPP_INFO(this->get_logger(), 
      "Latency: %.2f ms (min: %.2f, max: %.2f, avg: %.2f)",
      avg_latency, min_latency, max_latency, avg_latency);
    RCLCPP_INFO(this->get_logger(), 
      "Total data: %.2f MB", total_bytes_ / (1024.0 * 1024.0));
    RCLCPP_INFO(this->get_logger(), "================================");
  }

  // Subscribers
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr string_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr pose_array_sub_;
  
  // Timer
  rclcpp::TimerBase::SharedPtr report_timer_;
  
  // Parameters
  std::string topic_name_;
  int window_size_;
  double report_interval_;
  
  // Statistics
  rclcpp::Time start_time_;
  rclcpp::Time last_message_time_;
  size_t message_count_;
  size_t total_bytes_;
  
  // Rolling statistics
  std::deque<double> latencies_;
  std::deque<double> message_rates_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<BandwidthSubscriber>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
} 