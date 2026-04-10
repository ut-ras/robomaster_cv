#include <memory>
#include <algorithm>
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include <opencv2/opencv.hpp>

using std::placeholders::_1;

class LidarVisualizer : public rclcpp::Node
{
public:
  LidarVisualizer() : Node("lidar_visualizer")
  {
    // Subscribe to the /scan topic with a queue size of 10
    subscription_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
      "/scan", 
      10, 
      std::bind(&LidarVisualizer::scan_callback, this, _1));

    RCLCPP_INFO(this->get_logger(), "Lidar visualizer node started. Listening to /scan...");
  }

private:
  void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg) const
  {
    // 1. Create a blank black image (500x500 pixels)
    int width = 500;
    int height = 500;
    cv::Mat img = cv::Mat::zeros(height, width, CV_8UC3);

    // 2. Draw the robot at the center (Green dot)
    cv::Point center(width / 2, height / 2);
    cv::circle(img, center, 3, cv::Scalar(0, 255, 0), -1);

    // Scale factor: 1 meter = 50 pixels
    double scale = 50.0;

    // 3. Loop through the laser ranges
    // The angle starts at angle_min and increments by angle_increment
    float angle = msg->angle_min;

    for (size_t i = 0; i < msg->ranges.size(); ++i) {
      float r = msg->ranges[i];

      // Filter out invalid ranges (too close or too far)
      if (r > msg->range_min && r < msg->range_max) {
        
        // POLAR TO CARTESIAN CONVERSION
        // x = r * cos(theta)
        // y = r * sin(theta)
        double x = r * std::cos(angle);
        double y = r * std::sin(angle);

        // MAP TO PIXEL COORDINATES
        // Image Y grows DOWN, so we subtract y from center
        // Image X grows RIGHT, so we add x to center (rotated -90 for standard view)
        int px = static_cast<int>(center.x + (y * scale)); // Swap x/y for screen orientation
        int py = static_cast<int>(center.y - (x * scale));

        // Draw the point (Red dot)
        if (px >= 0 && px < width && py >= 0 && py < height) {
           img.at<cv::Vec3b>(py, px) = cv::Vec3b(0, 0, 255); // Red (BGR format)
        }
      }
      
      // Increment angle for next point
      angle += msg->angle_increment;
    }

    // 4. Show the image
    cv::imshow("Lidar Visualization", img);
    cv::waitKey(1); // REQUIRED for OpenCV to update the window
  }

  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subscription_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LidarVisualizer>());
  rclcpp::shutdown();
  return 0;
}