#include <cstdio>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <realsense2_camera_msgs/msg/rgbd.hpp>

using std::placeholders::_1;

constexpr char INPUT_TOPIC_NAME[] = "/camera/camera/rgbd";
constexpr char OUTPUT_TOPIC_NAME[] = "/preprocessing/rgbd";

class PreprocessingNode : public rclcpp::Node {
  rclcpp::Publisher<realsense2_camera_msgs::msg::RGBD>::SharedPtr rgbd_publisher_;
  rclcpp::Subscription<realsense2_camera_msgs::msg::RGBD>::SharedPtr subscription_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr a;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr b;

public:
  PreprocessingNode() : Node("preprocessing_node") {
    rgbd_publisher_ = this->create_publisher<realsense2_camera_msgs::msg::RGBD>(OUTPUT_TOPIC_NAME, 1);
    subscription_ = this->create_subscription<realsense2_camera_msgs::msg::RGBD>(INPUT_TOPIC_NAME, 1, std::bind(&PreprocessingNode::rgbd_callback, this, _1));
    a = this->create_publisher<sensor_msgs::msg::Image>("/image", 10);
    b = this->create_subscription<sensor_msgs::msg::Image>("/image_rect_color", 10, std::bind(&PreprocessingNode::c, this, _1));
  }

private:
  void rgbd_callback(const realsense2_camera_msgs::msg::RGBD::SharedPtr msg) {
      auto rgb = msg->rgb;
      auto depth = msg->depth;
      // TODO: Perform Preprocessing here
      rgbd_publisher_->publish(*msg);
  }
  void c(const sensor_msgs::msg::Image::SharedPtr msg) {
      a->publish(*msg);
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

