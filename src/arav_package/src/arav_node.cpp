#include <cstdio>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

using std::placeholders::_1;

class AravNode : public rclcpp::Node {
public:
  AravNode() : Node("AravNode") {
    printf("hello world arav_package package\n");
    subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
    "image_raw", 10, std::bind(&AravNode::topic_callback, this, _1));
  }

private:
  void topic_callback(const sensor_msgs::msg::Image::SharedPtr msg) const
  {
        cv_bridge::CvImagePtr cv_ptr;
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        cv::Mat cv_image = cv_ptr->image;

        cv::Mat gray_image;
        cv::cvtColor(cv_image, gray_image, cv::COLOR_BGR2GRAY);
        cv::imwrite("original_image.jpg", cv_image);
        cv::imwrite("grayscale_image.jpg", gray_image);

  }

  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_;
};

int main(int argc, char ** argv)
{
  (void) argc;
  (void) argv;

  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<AravNode>());
  rclcpp::shutdown();

  return 0;
}
