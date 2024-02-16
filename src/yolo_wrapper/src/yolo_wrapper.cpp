#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include <realsense2_camera_msgs/msg/rgbd.hpp>

using namespace std::chrono_literals;

constexpr char INPUT_TOPIC_NAME[] = "/preprocessing/rgbd";
constexpr char OUTPUT_TOPIC_NAME[] = "/yolo/output";

/* This example creates a subclass of Node and uses std::bind() to register a
* member function as a callback from the timer. */

class YOLOWrapper : public rclcpp::Node
{
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
  rclcpp::Publisher<realsense2_camera_msgs::msg::RGBD>::SharedPtr subscriber_;
  size_t count_;

  public:
    YOLOWrapper()
    : Node("yolo_wrapper"), count_(0)
    {
      publisher_ = this->create_publisher<std_msgs::msg::String>(OUTPUT_TOPIC_NAME, 10);
      subscriber_ = this->create_publisher<realsense2_camera_msgs::msg::RGBD>(INPUT_TOPIC_NAME, 1);
      timer_ = this->create_wall_timer(500ms, std::bind(&YOLOWrapper::timer_callback, this));
    }

  private:
    void timer_callback()
    {
      auto message = std_msgs::msg::String();
      message.data = "Hello, world! " + std::to_string(count_++);
      RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
      publisher_->publish(message);
    }
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<YOLOWrapper>());
  rclcpp::shutdown();
  return 0;
}