#include <cstdio>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>


class DevANode : public rclcpp::Node {
  public:
    DevANode() : Node("DevA_Node") {
      img_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
        "image_raw",
        rclcpp::QoS(1),
        std::bind(&DevANode::imageCallback, this, std::placeholders::_1)
      );
    }
    void imageCallback(sensor_msgs::msg::Image::SharedPtr msg) {
      RCLCPP_INFO(get_logger(), "The height is %d and width is %d", msg->height, msg->width); 
    }
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr img_sub_;
};

int main(int argc, char ** argv)
{
  (void) argc;
  (void) argv;

  printf("hello world deva_package package\n");

  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<DevANode>());
  rclcpp::shutdown(); 

  return 0;
}
