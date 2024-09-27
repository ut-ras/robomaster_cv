#include <cstdio>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/imgcodecs.hpp>

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
      cv_bridge::CvImagePtr cv_ptr;
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
      cv::Mat src = cv_ptr->image;

      cv::Mat mat;
      cv::GaussianBlur(src, mat, cv::Size(101, 101), 0, 0);
      cv::imwrite("original.jpg", src);
      cv::imwrite("processed.jpg", mat);
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
