#include <rclcpp/rclcpp.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

class DamianNode : public rclcpp::Node {
public:
  DamianNode() : Node("damian_node") {
    RCLCPP_INFO(this->get_logger(), "DamianNode started!");
    processImage();  // Directly process `image.jpg`
  }

private:
  void processImage() {
    // Load the image
    cv::Mat image = cv::imread("image.jpg");

    if (image.empty()) {
      RCLCPP_ERROR(this->get_logger(), "Failed to load image.jpg!");
      return;
    }

    RCLCPP_INFO(this->get_logger(), "Processing image.jpg...");

    // Save the original image
    cv::imwrite("original_image.jpg", image);

    // Apply grayscale conversion
    cv::Mat processed_image;
    cv::cvtColor(image, processed_image, cv::COLOR_BGR2GRAY);

    // Save processed image
    cv::imwrite("processed_image.jpg", processed_image);

    // Display images
    RCLCPP_INFO(this->get_logger(), "Saving images (GUI disabled)...");

    cv::imwrite("original_image.jpg", image);
    cv::imwrite("processed_image.jpg", processed_image);
    
    RCLCPP_INFO(this->get_logger(), "Images saved successfully!");
    

    RCLCPP_INFO(this->get_logger(), "Image processing completed!");
  }
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<DamianNode>();
  rclcpp::shutdown();
  return 0;
}
