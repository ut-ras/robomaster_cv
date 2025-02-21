#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

class ImageProcessor : public rclcpp::Node {
  public:
      ImageProcessor() : Node("image_processor") {
          subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
              "/image_raw", 10, std::bind(&ImageProcessor::imageCallback, this, std::placeholders::_1));

          publisher_ = this->create_publisher<sensor_msgs::msg::Image>("/processed_image", 10);

          RCLCPP_INFO(this->get_logger(), "Image Processor Node Started...");
      }

  private:
      void imageCallback(const sensor_msgs::msg::Image::SharedPtr msg) {
          try {
              cv::Mat original_image = cv_bridge::toCvCopy(msg, "bgr8")->image;

              cv::Mat gray_image, edges;
              cv::cvtColor(original_image, gray_image, cv::COLOR_BGR2GRAY);
              cv::Canny(gray_image, edges, 50, 150);

              cv::imwrite("/robomaster_cv/original_image.jpg", original_image);
              cv::imwrite("/robomaster_cv/processed_image.jpg", edges);

              RCLCPP_INFO(this->get_logger(), "Saved images: original and processed.");

              cv_bridge::CvImage processed_msg;
              processed_msg.header = msg->header;
              processed_msg.encoding = "mono8";  
              processed_msg.image = edges;

              publisher_->publish(*processed_msg.toImageMsg());
          }
          catch (const cv_bridge::Exception &e) {
              RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
          }
      }
      rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_;
      rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_;
  };

  int main(int argc, char **argv) {
      rclcpp::init(argc, argv);
      rclcpp::spin(std::make_shared<ImageProcessor>());
      rclcpp::shutdown();
      return 0;
  }
