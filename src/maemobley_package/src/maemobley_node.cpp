#include <cstdio>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/opencv.hpp>
using std::placeholders::_1;

class MaeNode : public rclcpp::Node {

public:
  MaeNode() : Node("MaeNode") {
    printf("hello world maemobley_package package\n");
    subscription_ = this->create_subscription<sensor_msgs::msg::Image>("image_raw", 10, std::bind(&MaeNode::topic_callback, this, _1));
  }

private:
    void topic_callback(const sensor_msgs::msg::Image & msg) const
    {
      RCLCPP_INFO(this->get_logger(), "I heard: '%d %d'", msg.width, msg.height);

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
      cv::Size centre = cv::Size(1,1);


      cv::blur(cv_ptr->image, mat, centre);

      cv::imwrite("original.png", cv_ptr->image);
      cv::imwrite("blur.png", mat);
    }

    
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_;
};

int main(int argc, char ** argv)
{
  (void) argc;
  (void) argv;

  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MaeNode>());
  rclcpp::shutdown();

  return 0;
}
