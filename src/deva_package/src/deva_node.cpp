#include <cstdio>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include "opencv2/imgcodecs.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/imgproc.hpp"
#include <cv_bridge/cv_bridge.h>

using namespace cv;

class DevANode : public rclcpp::Node {
  public:
  // default, no-parameter constructor
    DevANode() : Node("DevA_Node") {
      // creates a subcription to the image and sends image to method imageCallback 
      img_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
        "image_raw",
        rclcpp::QoS(1),
        std::bind(&DevANode::imageCallback, this, std::placeholders::_1)
      );
    }
    // processes the image from ros type to opencv type
    // applies sobel operator for edge detection
    void imageCallback(sensor_msgs::msg::Image::SharedPtr msg) {
      // processing 
      cv_bridge::CvImagePtr cv_ptr;
      try{
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);

      } catch (cv_bridge::Exception& e) {
        RCLCPP_ERROR(get_logger() ,"cv_bridge exception: %s", e.what());
        return;
      }
    // get the matrix of pixels from cv_ptr
    Mat image = cv_ptr->image;
    // initialize ncessary variables
    Mat src, src_gray;
    Mat grad;
    const String window_name = "Sobel Demo - Simple Edge Detector";
    int ksize = 3;
    int scale = 1;
    int delta = 0;
    int ddepth = CV_16S;


    // Remove noise by blurring with a Gaussian filter ( kernel size = 3 )
    GaussianBlur(image, src, Size(3, 3), 0, 0, BORDER_DEFAULT);

    // Convert the image to grayscale
    cvtColor(src, src_gray, COLOR_BGR2GRAY);
    
    Mat grad_x, grad_y;
    Mat abs_grad_x, abs_grad_y;
    
    Sobel(src_gray, grad_x, ddepth, 1, 0, ksize, scale, delta, BORDER_DEFAULT);
 
    Sobel(src_gray, grad_y, ddepth, 0, 1, ksize, scale, delta, BORDER_DEFAULT);

    // converting back to CV_8U
    convertScaleAbs(grad_x, abs_grad_x);
    convertScaleAbs(grad_y, abs_grad_y);

    addWeighted(abs_grad_x, 0.5, abs_grad_y, 0.5, 0, grad);

    cv::imwrite("/robomaster_cv/original.jpg", cv_ptr->image);
    cv::imwrite("/robomaster_cv/edges.jpg", grad);



      // RCLCPP_INFO(get_logger(), "The height is %d and width is %d", msg->height, msg->width); 
    }
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr img_sub_;

};

int main(int argc, char ** argv)
{
  (void) argc;
  (void) argv;

  printf("hello world deva_package package\n");

  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<DevANode>());
  rclcpp::shutdown(); 

  return 0;
}
