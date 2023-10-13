#include <cstdio>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <realsense2_camera_msgs/msg/rgbd.hpp>

using std::placeholders::_1;

class PreprocessingNode : public rclcpp::Node {
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr rgb_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr depth_publisher_;
  rclcpp::Subscription<realsense2_camera_msgs::msg::RGBD>::SharedPtr subscription_;

public:
  PreprocessingNode() : Node("preprocessing_node") {
    rgb_publisher_ = this->create_publisher<sensor_msgs::msg::Image>("/preprocessing/rgb", 10);
    depth_publisher_ = this->create_publisher<sensor_msgs::msg::Image>("/preprocessing/depth", 10);
    subscription_ = this->create_subscription<realsense2_camera_msgs::msg::RGBD>("/camera/camera/rgbd", 10, std::bind(&PreprocessingNode::rgbd_callback, this, _1));
  }

private:
  void rgbd_callback(const realsense2_camera_msgs::msg::RGBD::SharedPtr msg) {
      auto rgb = msg->rgb;
      auto depth = msg->depth;
      rgb_publisher_->publish(rgb);
      depth_publisher_->publish(depth);
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

