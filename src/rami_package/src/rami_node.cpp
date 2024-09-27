#include <cstdio>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui/highgui.hpp>  // Optional for image display
using std::placeholders::_1;

class RamiNode : public rclcpp::Node {
public:
  RamiNode() : Node("rami_node") {
    // Inform the console that the node is up
    RCLCPP_INFO(this->get_logger(), "RamiNode has been started.");    
    
    subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
    "image_raw", 10, std::bind(&RamiNode::topic_callback, this, _1));  
  }

private:
  // Callback function triggered when a message is received on the "image_raw" topic
  void topic_callback(const sensor_msgs::msg::Image::SharedPtr msg) const
  {
    // Convert the ROS2 image message to an OpenCV image using cv_bridge
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
      RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
      return;
    }

    // Edge detection using Canny algorithm
    cv::Mat edges;
    cv::Canny(cv_ptr->image, edges, 80, 80 * 3, 3);

    // Save the original and processed images to disk
    cv::imwrite("original.jpg", cv_ptr->image);
    cv::imwrite("edges.jpg", edges);

    // Optional: Show the original and edge-detected images in a window
    // cv::imshow("Original Image", cv_ptr->image);
    // cv::imshow("Edge-detected Image", edges);
    // cv::waitKey(1);

    RCLCPP_INFO(this->get_logger(), "Image processed and saved.");
  }

  // Subscriber to the "image_raw" topic
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_;
};

int main(int argc, char ** argv)
{
  // Initialize the ROS 2 system
  rclcpp::init(argc, argv);

  // Create an instance of the ArthurNode and spin it to handle callbacks
  rclcpp::spin(std::make_shared<RamiNode>());

  // Shutdown the ROS 2 system when finished
  rclcpp::shutdown();

  return 0;
}

