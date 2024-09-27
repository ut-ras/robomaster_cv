#include <cstdio>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp> // Image type ROS message
// #include <image_transport/image_transport.hpp> // Image transport node
#include <cv_bridge/cv_bridge.h> // Bridge converter
// #include <sensor_msgs/image_encodings.h> // Encodings like BGR and Gray
#include <opencv2/imgproc/imgproc.hpp> // ImgProc
#include <opencv2/highgui/highgui.hpp> // GUI
#include <opencv2/imgcodecs.hpp> // Img write

static const std::string WINDOW_NAME = "Oye Window";
class HugoMirelesNode : public rclcpp::Node {
  // ament_target_dependencies(HugoMireles_node rclcpp sensor_msgs)
  public :
    HugoMirelesNode() : Node("HugoMireles_node") {
      printf("hello world HugoMireles_package package\n");
      subscription_ = this->create_subscription<sensor_msgs::msg::Image>("image_raw", 10, std::bind(&HugoMirelesNode::hugo_callback, this, std::placeholders::_1));
      // cv::namedWindow(WINDOW_NAME);
    }

    ~HugoMirelesNode() {
      // cv::destroyWindow(WINDOW_NAME);
      
    }

  private:
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_;

    void hugo_callback(const sensor_msgs::msg::Image & msg) const{
      cv_bridge::CvImagePtr img_ptr;
      // Try to convert image
      try {
        img_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
      } catch(cv_bridge::Exception& e) {
        RCLCPP_ERROR(get_logger(), "cv_bridge exception: %s", e.what());
        return;
      }

      cv::Mat mat;
      // cv::waitKey(s30);
      cv::blur(img_ptr->image, mat, cv::Size(7,7));
      // cv::imshow(WINDOW_NAME, mat);
      cv::imwrite("original.jpg", img_ptr->image);
      cv::imwrite("blurredImg.jpg", mat);

      RCLCPP_INFO(this->get_logger(), "I heard: '%d %d'", msg.width, msg.height);
    }
};

int main(int argc, char ** argv)
{
  (void) argc;
  (void) argv;

  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<HugoMirelesNode>());
  rclcpp::shutdown();
  return 0;
}
