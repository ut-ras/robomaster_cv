#include <cstdio>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>
using std::placeholders::_1;

class Neel : public rclcpp::Node{

public:
  Neel() : Node("Neel"){
    printf("hello world NeelPerumandla package\n");
    subscription_ = this->create_subscription<sensor_msgs::msg::Image>("image_raw", rclcpp::QoS(1), std::bind(&Neel::topic_callback, this, _1));
  }

private:
  void topic_callback(const sensor_msgs::msg::Image::SharedPtr msg) const 
  {
    cv_bridge::CvImagePtr cv_ptr;
    cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    cv::Mat src;
    src = cv_ptr->image;
    
    cv::Mat mat;
    cv::GaussianBlur(src, src, cv::Size(3, 3), 0, 0, cv::BORDER_DEFAULT);
    cv::cvtColor(src, src, cv::COLOR_BGR2GRAY);
    cv::Laplacian(src, mat, CV_16S, 3, 1, 0, cv::BORDER_DEFAULT);

    convertScaleAbs(mat, mat);
    cv::imwrite("previous.jpg", src);
    cv::imwrite("current.jpg", mat);

  }
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_;
    
};

int main(int argc, char ** argv)
{
  (void) argc;
  (void) argv;

  
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Neel>());
  rclcpp::shutdown();

  
  return 0;
}
