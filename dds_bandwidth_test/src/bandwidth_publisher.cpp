#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <chrono>
#include <memory>
#include <string>
#include <vector>

class BandwidthPublisher : public rclcpp::Node
{
public:
  BandwidthPublisher() : Node("bandwidth_publisher")
  {
    // Parameters
    this->declare_parameter("message_type", "string");
    this->declare_parameter("message_size", 8192);  // 8MB default for high bandwidth
    this->declare_parameter("publish_rate", 1000.0); // 1000Hz for high throughput
    this->declare_parameter("duration", 60.0);
    this->declare_parameter("topic_name", "bandwidth_test");

    message_type_ = this->get_parameter("message_type").as_string();
    message_size_ = this->get_parameter("message_size").as_int();
    publish_rate_ = this->get_parameter("publish_rate").as_double();
    duration_ = this->get_parameter("duration").as_double();
    topic_name_ = this->get_parameter("topic_name").as_string();

    // Create publishers based on message type
    if (message_type_ == "string") {
      string_pub_ = this->create_publisher<std_msgs::msg::String>(
        topic_name_ + "_string", 1000);  // Larger queue for high frequency
    } else if (message_type_ == "image") {
      image_pub_ = this->create_publisher<sensor_msgs::msg::Image>(
        topic_name_ + "_image", 1000);  // Larger queue for high frequency
    } else if (message_type_ == "pose_array") {
      pose_array_pub_ = this->create_publisher<geometry_msgs::msg::PoseArray>(
        topic_name_ + "_pose_array", 1000);  // Larger queue for high frequency
    }

    // Create timer
    timer_ = this->create_wall_timer(
      std::chrono::duration<double>(1.0 / publish_rate_),
      std::bind(&BandwidthPublisher::timer_callback, this));

    // Statistics
    start_time_ = this->now();
    message_count_ = 0;
    total_bytes_ = 0;

    RCLCPP_INFO(this->get_logger(), 
      "Starting bandwidth test: %s messages, %d KB, %.1f Hz, %.1f seconds",
      message_type_.c_str(), message_size_, publish_rate_, duration_);
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
        "Test completed: %ld messages, %.2f msg/s, %.2f MB/s",
        message_count_, avg_rate, avg_bandwidth / (1024 * 1024));
      
      rclcpp::shutdown();
      return;
    }

    message_count_++;
    
    if (message_type_ == "string") {
      publish_string_message();
    } else if (message_type_ == "image") {
      publish_image_message();
    } else if (message_type_ == "pose_array") {
      publish_pose_array_message();
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

  void publish_string_message()
  {
    auto msg = std_msgs::msg::String();
    // Interpret message_size_ as KB to be consistent with other message types
    msg.data = generate_string_data(message_size_ * 1024);
    string_pub_->publish(msg);
    total_bytes_ += msg.data.size();
  }

  void publish_image_message()
  {
    auto msg = sensor_msgs::msg::Image();
    msg.header.stamp = this->now();
    msg.header.frame_id = "camera_frame";
    msg.height = 1080;
    msg.width = 1920;
    msg.encoding = "rgb8";
    msg.step = msg.width * 3;
    
    // Calculate how many images we need to reach target size
    int target_size = message_size_ * 1024; // Convert KB to bytes
    int image_size = msg.height * msg.width * 3;
    int num_images = std::max(1, target_size / image_size);
    
    msg.data.resize(num_images * image_size, 0);
    msg.height = 1080 * num_images;
    
    image_pub_->publish(msg);
    total_bytes_ += msg.data.size();
  }

  void publish_pose_array_message()
  {
    auto msg = geometry_msgs::msg::PoseArray();
    msg.header.stamp = this->now();
    msg.header.frame_id = "world";
    
    // Calculate number of poses needed
    int pose_size = 56; // Approximate size of a pose in bytes
    int num_poses = message_size_ * 1024 / pose_size;
    num_poses = std::max(1, num_poses);
    
    msg.poses.resize(num_poses);
    for (auto& pose : msg.poses) {
      pose.position.x = 1.0;
      pose.position.y = 2.0;
      pose.position.z = 3.0;
      pose.orientation.w = 1.0;
      pose.orientation.x = 0.0;
      pose.orientation.y = 0.0;
      pose.orientation.z = 0.0;
    }
    
    pose_array_pub_->publish(msg);
    total_bytes_ += num_poses * pose_size;
  }

  std::string generate_string_data(int size_bytes)
  {
    std::string data;
    data.reserve(size_bytes);
    
    // Use a larger pattern for better performance
    const std::string pattern = "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789";
    int pattern_size = pattern.size();
    
    for (int i = 0; i < size_bytes; ++i) {
      data += pattern[i % pattern_size];
    }
    
    return data;
  }

  // Publishers
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr string_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr pose_array_pub_;
  
  // Timer
  rclcpp::TimerBase::SharedPtr timer_;
  
  // Parameters
  std::string message_type_;
  int message_size_;
  double publish_rate_;
  double duration_;
  std::string topic_name_;
  
  // Statistics
  rclcpp::Time start_time_;
  size_t message_count_;
  size_t total_bytes_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<BandwidthPublisher>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
} 