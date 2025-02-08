#include <cstdio>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>


using std::placeholders::_1;

class DominicNode : public rclcpp::Node {

public:
  DominicNode() : Node("DominicNode") {
    printf("hello world dominic_package package\n");
    subscription_ = this->create_subscription<sensor_msgs::msg::Image>
    ( "image_raw", 10, std::bind(&DominicNode::image_callback, this, _1));
  }
private: 
  void image_callback(const sensor_msgs::msg::Image & msg) const
    {
      RCLCPP_INFO(this->get_logger(), "I heard: '%d %d'", msg.width, msg.height);
    }
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_;
};

int main(int argc, char ** argv)
{
  (void) argc;
  (void) argv;

  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<DominicNode>());
  rclcpp::shutdown();

  return 0;
}
