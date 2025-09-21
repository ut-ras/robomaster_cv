#include "rclcpp/rclcpp.hpp"
#include "vision_msgs/msg/detection2_d_array.hpp"
#include "vision_msgs/msg/detection2_d.hpp"
#include "std_msgs/msg/header.hpp"
#include <limits>
#include <cmath>

class TargetSelectorNode : public rclcpp::Node
{
public:
    TargetSelectorNode() : Node("target_selector_node")
    {
        // Declare and get camera resolution parameters
        this->declare_parameter("image_width", 1280.0);
        this->declare_parameter("image_height", 720.0);
        this->get_parameter("image_width", image_width_);
        this->get_parameter("image_height", image_height_);
        center_x_ = image_width_ / 2.0;
        center_y_ = image_height_ / 2.0;

        // Set up publishers and subscribers
        detection_sub_ = this->create_subscription<vision_msgs::msg::Detection2DArray>(
            "/detections", 10,
            std::bind(&TargetSelectorNode::detection_callback, this, std::placeholders::_1));

        target_pub_ = this->create_publisher<vision_msgs::msg::Detection2D>("/target", 10);

        RCLCPP_INFO(this->get_logger(), "Target Selector Node initialized with resolution %.0fx%.0f", image_width_, image_height_);
    }

private:
    rclcpp::Subscription<vision_msgs::msg::Detection2DArray>::SharedPtr detection_sub_;
    rclcpp::Publisher<vision_msgs::msg::Detection2D>::SharedPtr target_pub_;

    double image_width_;
    double image_height_;
    double center_x_;
    double center_y_;

    void detection_callback(const vision_msgs::msg::Detection2DArray::SharedPtr msg)
    {
        if (msg->detections.empty())
        {
            RCLCPP_DEBUG(this->get_logger(), "No detections received.");
            return;
        }

        double min_distance = std::numeric_limits<double>::max();
        vision_msgs::msg::Detection2D best_detection;

        for (const auto &detection : msg->detections)
        {
            double dx = detection.bbox.center.position.x - center_x_;
            double dy = detection.bbox.center.position.y - center_y_;
            double distance = dx * dx + dy * dy;

            if (distance < min_distance)
            {
                min_distance = distance;
                best_detection = detection;
            }
        }

        best_detection.header.stamp = this->get_clock()->now();
        best_detection.header.frame_id = msg->header.frame_id;

        target_pub_->publish(best_detection);

        RCLCPP_INFO(this->get_logger(), "Published best detection at (%.2f, %.2f)",
                     best_detection.bbox.center.position.x,
                     best_detection.bbox.center.position.y);
    }
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<TargetSelectorNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
