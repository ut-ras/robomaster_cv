#include <cstdio>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>

class PrajithNode : public rclcpp::Node {
  public:
    PrajithNode() : Node("prajithv_node"), last_msg_{nullptr} {
      sub_ = this->create_subscription<sensor_msgs::msg::Image>(
        "image_raw",
        rclcpp::QoS(1),
        std::bind(&PrajithNode::prajithvCallback, this, std::placeholders::_1)
      );
    }

    void prajithvCallback(sensor_msgs::msg::Image::SharedPtr msg) {
      last_msg_ = msg;
      RCLCPP_INFO(
        get_logger(), 
        "Width: %d, Height: %d",
        msg->width,
        msg->height
      );
    }

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_;
    sensor_msgs::msg::Image::SharedPtr last_msg_;
};

int main(int argc, char ** argv)
{
  (void) argc;
  (void) argv;

  rclcpp::init(argc, argv);

  auto node = std::make_shared<PrajithNode>();

  RCLCPP_INFO(node->get_logger(), "prajithv node starting");

  rclcpp::spin(node);
  rclcpp::shutdown();

  return 0;
}
