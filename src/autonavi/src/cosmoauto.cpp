#include <chrono>
#include <memory>
#include <string>
#include <thread>

#include <cstdio>
#include <rclcpp/rclcpp.hpp>
#include "std_msgs/msg/int32_multi_array.hpp"

class cosmoauto : public rclcpp::Node
{
  public: 
  cosmoauto() : Node("cosmoauto") {
    health_subscriber_ = this->create_subscription<std_msgs::msg::Int32MultiArray>(
      "health", 10, 
      std::bind(&cosmoauto::health_callback, this, std::placeholders::_1));

    ammunition_subscriber_ = this->create_subscription<std_msgs::msg::Int32MultiArray>(
      "ammunition", 10,
      std::bind(&cosmoauto::ammunition_callback, this, std::placeholders::_1));
    
    status_publisher_ = this->create_publisher<std_msgs::msg::String>("status", 10);

  }

  std::vector<int> get_latest_ammunition() const {
    return latest_ammunition_;
  }
  
  std::vector<int> get_latest_health() const {
    return latest_health;
  }

  private:
  void health_callback(const std_msgs::msg::Int32MultiArray::SharedPtr msg)
  {
    latest_health = msg -> data;
    RCLCPP_INFO(this->get_logger(), "Saved new health data with %zu values", latest_health.size());
    publish_status();
  }

  void ammunition_callback(const std_msgs::msg::Int32MultiArray::SharedPtr msg)
  {
    RCLCPP_INFO(this->get_logger(), "Received ammunition with %zu values", msg->data.size());
    latest_ammunition_ = msg->data;
    publish_status();
  }


  void publish_status()
  {
    if (latest_health_.empty() || latest_ammunition_.empty()) {
      return;  // don’t publish until we have data from both
    }

    // For simplicity, assume first element is the "score"
    int health_value = latest_health_[0];
    int ammunition_value = latest_ammunition_[0];
    

    // IMPLEMENT COORDINATES HERE
    int x_coordinate = 0;  // Replace with actual x coordinate
    int y_coordinate = 0;  // Replace with actual y coordinate
    auto message = std_msgs::msg::String();
    if (health_value >= HEALTH_THRESHOLD && ammunition_value >= AMMUNITION_THRESHOLD) {
      message.data = "STAY";
    } else {
      message.data = "LEAVE";
    }

    RCLCPP_INFO(this->get_logger(), "Publishing status: %s", message.data.c_str());
    status_publisher_->publish(message);
  }

  rclcpp::Subscription<std_msgs::msg::Int32MultiArray>::SharedPtr health_subscriber_;
  rclcpp::Subscription<std_msgs::msg::Int32MultiArray>::SharedPtr ammunition_subscriber_;
  std::vector<int> latest_health;
  std::vector<int> latest_ammunition_;

  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_publisher_;
  static constexpr int HEALTH_THRESHOLD = 50;
  static constexpr int AMMUNITION_THRESHOLD = 10;


}
int main(int argc, char ** argv)
{
  (void) argc;
  (void) argv;
  rclcpp::init(argc, argv);
  auto node = std::make_shared<cosmoauto>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
