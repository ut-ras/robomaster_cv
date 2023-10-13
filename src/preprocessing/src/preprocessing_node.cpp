#include <cstdio>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <realsense2_camera_msgs/msg/rgbd.hpp>

class PreprocessingNode : public rclcpp::Node {
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;

public:
  PreprocessingNode() : Node("preprocessing_node") {
    publisher_ = this->create_publisher<std_msgs::msg::String>("topic", 10);
    subscription_ = this->create_subscription<realsense2_camera_msgs::msg::RGBD>("/camera/camera/rgbd", 10, std::bind(&PreprocessingNode::rgbd_callback, this, _1));
  }

private:
  void rgbd_callback(const realsense2_camera_msgs::msg::RGBD::SharedPtr msg) {

  }
};

int main(int argc, char ** argv)
{
  (void) argc;
  (void) argv;

  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PreprocessingNode>());
  rclcpp::shutdown();

  return 0;
}

