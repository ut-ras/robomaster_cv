#include <cstdio>
#include <rclcpp/rclcpp.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include "custom_synchronizer.h"
#include <sensor_msgs/msg/image.hpp>
#include <vision_msgs/msg/detection2_d_array.hpp>

class VisualizerNode : public rclcpp::Node {
  message_filters::Subscriber<sensor_msgs::msg::Image> image_subscriber_;
  message_filters::Subscriber<sensor_msgs::msg::Image> depth_subscriber_;
  message_filters::Subscriber<vision_msgs::msg::Detection2DArray> detections_subscriber_;
  message_filters::CustomSynchronizer<sensor_msgs::msg::Image, sensor_msgs::msg::Image, vision_msgs::msg::Detection2DArray> sync;


public: 
  VisualizerNode() : Node("visualizer"), sync(image_subscriber_, depth_subscriber_, detections_subscriber_) {
    image_subscriber_.subscribe(this, "/image");
    depth_subscriber_.subscribe(this, "/robot/rs2/depth/image_rect_raw");
    detections_subscriber_.subscribe(this, "/detections_output");
    sync.registerCallback(std::bind(&VisualizerNode::callback, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));
  }

private:
  void callback(const sensor_msgs::msg::Image::ConstSharedPtr& img, const sensor_msgs::msg::Image::ConstSharedPtr& depth, const vision_msgs::msg::Detection2DArray::ConstSharedPtr& detections) {
    RCLCPP_INFO(this->get_logger(), "WOOOO!!!!!");
  }
};

int main(int argc, char ** argv)
{
  (void) argc;
  (void) argv;

  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<VisualizerNode>());
  rclcpp::shutdown();

  return 0;
}
