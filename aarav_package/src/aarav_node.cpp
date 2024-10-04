#include <cstdio>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

using std::placeholders::_1;

class aaravnode : public rclcpp::Node {
public:
  aaravnode() : Node("aaravnode") {
    printf("hello world aarav_package\n");
    subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
    "image_raw", 10, std::bind(&aaravnode::topic_callback, this, _1));
  }

private:
  void topic_callback(const sensor_msgs::msg::Image::SharedPtr msg) const
  {
        cv_bridge::CvImagePtr cv_ptr;
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        cv::Mat cv_image = cv_ptr->image;

        cv::Mat gray_image;
        cv::cvtColor(cv_image, gray_image, cv::COLOR_BGR2GRAY);
        cv::imwrite("original_color.webp", cv_image);
        cv::imwrite("grayscale_color.webp", gray_image);

  }

  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_;
};

int main(int argc, char ** argv)
{
  (void) argc;
  (void) argv;

  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<aaravnode>());
  rclcpp::shutdown();

  return 0;
}