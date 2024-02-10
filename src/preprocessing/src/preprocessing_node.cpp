#include <cstdio>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <realsense2_camera_msgs/msg/rgbd.hpp>

using std::placeholders::_1;

constexpr char YOLO_INPUT_TOPIC_NAME[] = "/image";

class PreprocessingNode : public rclcpp::Node {
  rclcpp::Publisher<realsense2_camera_msgs::msg::RGBD>::SharedPtr rgbd_publisher_;
  rclcpp::Subscription<realsense2_camera_msgs::msg::RGBD>::SharedPtr subscription_;

public:
  PreprocessingNode() : Node("preprocessing_node") {
    rgbd_publisher_ = this->create_publisher<realsense2_camera_msgs::msg::RGBD>("/preprocessing/rgbd", 1);
    subscription_ = this->create_subscription<realsense2_camera_msgs::msg::RGBD>("/camera/camera/rgbd", 1, std::bind(&PreprocessingNode::rgbd_callback, this, _1));
  }

private:
  void rgbd_callback(const realsense2_camera_msgs::msg::RGBD::SharedPtr msg) {
      auto rgb = msg->rgb;
      auto depth = msg->depth;
      // TODO: Perform Preprocessing here
      rgbd_publisher_->publish(*msg);
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

