#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include <realsense2_camera_msgs/msg/rgbd.hpp>

using namespace std::chrono_literals;
using std::placeholders::_1;

constexpr char INPUT_TOPIC_NAME[] = "/preprocessing/rgbd"; // realsense2_camera_msgs/msg/RGBD
constexpr char YOLO_INPUT_TOPIC_NAME[] = "/image"; // sensor_msgs/msg/Image
constexpr char YOLO_OUTPUT_TOPIC_NAME[] = "/detections_output"; // vision_msgs/msg/Detection2DArray
constexpr char YOLO_OUTPUT_IMAGE_TOPIC_NAME[] = "/yolov8_processed_image"; // sensor_msgs/msg/Image
constexpr char OUTPUT_TOPIC_NAME[] = "/yolo/output"; // yolo_wrapper/msg/YoloOutput

/* This example creates a subclass of Node and uses std::bind() to register a
* member function as a callback from the timer. */

class YOLOWrapper : public rclcpp::Node
{
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr output_publisher_;
  rclcpp::Publisher<std_msgs::msg::Image>::SharedPtr yolo_publisher_;
  rclcpp::Subscription<realsense2_camera_msgs::msg::RGBD>::SharedPtr input_subscriber_;
  rclcpp::Subscription<realsense2_camera_msgs::msg::RGBD>::SharedPtr yolo_subscriber_;

  realsense2_camera_msgs::msg::RGBD::SharedPtr latest_received;
  realsense2_camera_msgs::msg::RGBD::SharedPtr latest_sent;

  public:
    YOLOWrapper()
    : Node("yolo_wrapper")
    {
      output_publisher_ = this->create_publisher<std_msgs::msg::String>(OUTPUT_TOPIC_NAME, 10);
      yolo_publisher_ = this->create_publisher<std_msgs::msg::Image>(YOLO_INPUT_TOPIC_NAME, 1);
      yolo_subscriber_ = this->create_subscription<realsense2_camera_msgs::msg::RGBD>(YOLO_OUTPUT_TOPIC_NAME, 1, std::bind(&YOLOWrapper::input_callback, this, _1));
      input_subscriber_ = this->create_subscription<realsense2_camera_msgs::msg::RGBD>(INPUT_TOPIC_NAME, 1, std::bind(&YOLOWrapper::input_callback, this, _1));
    }

  private:
    void input_callback(const realsense2_camera_msgs::msg::RGBD::SharedPtr msg)
    {
      if(latest_sent == nullptr) {
        latest_sent = msg;
        yolo_publisher_->publish(msg->rgb);
      } else {
        latest_received = msg;
      }
    }

    void yolo_callback(const realsense2_camera_msgs::msg::RGBD::SharedPtr msg) {

    }
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<YOLOWrapper>());
  rclcpp::shutdown();
  return 0;
}