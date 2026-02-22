#include <memory>
#include <algorithm>
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

using std::placeholders::_1;

class LidarScannerNode : public rclcpp::Node
{
public:
  LidarScannerNode() : Node("lidar_scanner_node")
  {
    // Subscribe to the /scan topic with a queue size of 10
    subscription_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
      "/scan", 
      10, 
      std::bind(&LidarScannerNode::scan_callback, this, _1));

    RCLCPP_INFO(this->get_logger(), "Lidar scanner node started. Listening to /scan...");
  }

private:
  void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg) const
  {
    float min_distance = msg->range_max;
    
    // Loop through the array of distances to find the closest object
    for (size_t i = 0; i < msg->ranges.size(); ++i) {
      float current_range = msg->ranges[i];
      
      // Filter out invalid readings (infinite or out of sensor bounds)
      if (current_range >= msg->range_min && current_range <= msg->range_max) {
        if (current_range < min_distance) {
          min_distance = current_range;
        }
      }
    }

    if (min_distance < msg->range_max) {
      RCLCPP_INFO(this->get_logger(), "Closest obstacle detected at: %.2f meters", min_distance);
    } else {
      RCLCPP_INFO(this->get_logger(), "No obstacles detected within range.");
    }
  }

  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subscription_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LidarScannerNode>());
  rclcpp::shutdown();
  return 0;
}