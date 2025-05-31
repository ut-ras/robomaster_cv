#include <cstdio>

// int main(int argc, char ** argv)
// {
//   (void) argc;
//   (void) argv;

//   printf("hello world tracking2 package\n");
//   return 0;
// }

#include <rclcpp/rclcpp.hpp>
#include <vision_msgs/msg/detection2_d_array.hpp>
#include <vision_msgs/msg/detection2_d.hpp>
#include <geometry_msgs/msg/pose2_d.hpp>

using std::placeholders::_1;
using vision_msgs::msg::Detection2DArray;

class DetectionListener : public rclcpp::Node {
public:
    DetectionListener() : Node("detection_listener") {
        subscription_ = this->create_subscription<Detection2DArray>(
            "detections", 10, std::bind(&DetectionListener::callback, this, _1));
        RCLCPP_INFO(this->get_logger(), "Detection listener node started.");
    }

private:
    rclcpp::Subscription<Detection2DArray>::SharedPtr subscription_;

    void callback(const Detection2DArray::SharedPtr msg) const {
        RCLCPP_INFO(this->get_logger(), "Received %zu detections", msg->detections.size());
        for (const auto &detection : msg->detections) {
            float x = detection.bbox.center.position.x;
            float y = detection.bbox.center.position.y;
            RCLCPP_INFO(this->get_logger(), "Detection - x: %.2f, y: %.2f", x, y);
        }
    }
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<DetectionListener>());
    rclcpp::shutdown();
    return 0;
}