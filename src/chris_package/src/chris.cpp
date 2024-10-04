#include <cstdio>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/imgcodecs.hpp>
using std::placeholders::_1;

class chrisNode : public rclcpp::Node {
public:
  chrisNode() : Node("chrisNode") {
    printf("hello world chris_package package\n");
    subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
    "image_raw", 10, std::bind(&chrisNode::topic_callback, this, _1));
    
  }

  private:
    void topic_callback(const sensor_msgs::msg::Image & msg) const
    {
      cv_bridge::CvImagePtr cv_ptr;
      try
      {
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
      }
      catch (cv_bridge::Exception& e)
      {
        RCLCPP_ERROR(get_logger(), "cv_bridge exception: %s", e.what());
        return;
      }

      cv::Mat mat;

      cv::Canny(cv_ptr->image, mat, 80, 80 * 3, 3);

      
      cv::imwrite("image.jpg", cv_ptr->image);
      cv::imwrite("image1.jpg", mat);
    }

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_;
};

int main(int argc, char ** argv)
{
  (void) argc;
  (void) argv;

  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<chrisNode>());
  rclcpp::shutdown();

  return 0;
}