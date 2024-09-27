#include <cstdio>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
using std::placeholders::_1;

class AdipuNode : public rclcpp::Node {

  public:
    AdipuNode() : Node("Adipu_node") {
      printf("Hello world Adipu_package package\n");
      subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
        "image_raw", 10, std::bind(&AdipuNode::topic_callback, this, _1)
      );
    }
  
  private:
    void topic_callback(const sensor_msgs::msg::Image::SharedPtr msg) const {
      RCLCPP_INFO(this->get_logger(), "I heard: '%d' x '%d'", msg->width, msg->height);
    }
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_;
};

int main(int argc, char ** argv)
{
  (void) argc;
  (void) argv;

  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<AdipuNode>());
  rclcpp::shutdown();
  
  return 0;
}
