#include <chrono>
#include <cstdio>
#include "rclcpp/rclcpp.hpp"
#include "object_log_msgs/msg/input.hpp"
#include "object-log/ObjectLog.h"

using namespace std::chrono_literals;

class ObjectLogPublisher : public rclcpp::Node
{
public:
  ObjectLogPublisher() : Node("object_log_pub"), count_(0)
  {
    publisher_ = this->create_publisher<object_log_msgs::msg::Input>("object_log/input", 10);
    timer_ = this->create_wall_timer(500ms, std::bind(&ObjectLogPublisher::timer_callback, this));
  }

private:
  void timer_callback()
  {
    auto message = object_log_msgs::msg::Input();
    message.id = 10;
    RCLCPP_INFO(this->get_logger(), "Publishing: %d", message.id);

    publisher_->publish(message);
  }
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<object_log_msgs::msg::Input>::SharedPtr publisher_;
  size_t count_;
};

int
main(int argc, char **argv)
{
  (void)argc;
  (void)argv;

  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ObjectLogPublisher>());
  rclcpp::shutdown();
  return 0;
}
