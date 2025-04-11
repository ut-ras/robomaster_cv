#include "rclcpp/rclcpp.hpp"
#include "vision_msgs/msg/detection2_d_array.hpp"
#include "vision_msgs/msg/detection2_d.hpp"
#include "std_msgs/msg/header.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include <limits>
#include <cmath>

class TargetSelectorNode : public rclcpp::Node
{
public:
    TargetSelectorNode() : Node("target_selector_node")
    {
        detection_sub_ = this->create_subscription<vision_msgs::msg::Detection2DArray>(
            "/detections", 10,
            std::bind(&TargetSelectorNode::detection_callback, this, std::placeholders::_1));

        target_pub_ = this->create_publisher<vision_msgs::msg::Detection2D>("/target", 10);

        marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("/target_marker", 10);

        // Set your actual camera resolution here (update if different)
        image_width_ = 1280.0;
        image_height_ = 720.0;
        center_x_ = image_width_ / 2.0;
        center_y_ = image_height_ / 2.0;

        RCLCPP_INFO(this->get_logger(), "Target Selector Node initialized.");
    }

private:
    rclcpp::Subscription<vision_msgs::msg::Detection2DArray>::SharedPtr detection_sub_;
    rclcpp::Publisher<vision_msgs::msg::Detection2D>::SharedPtr target_pub_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;

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

        // Set header stamp and frame ID
        best_detection.header.stamp = this->get_clock()->now();
        best_detection.header.frame_id = msg->header.frame_id;

        target_pub_->publish(best_detection);

        RCLCPP_INFO(this->get_logger(), "Published best detection at (%.2f, %.2f)",
                     best_detection.bbox.center.position.x,
                     best_detection.bbox.center.position.y);

        publish_marker(best_detection);
    }

    void publish_marker(const vision_msgs::msg::Detection2D &detection)
    {
        visualization_msgs::msg::Marker marker;
        marker.header.frame_id = detection.header.frame_id;
        marker.header.stamp = this->get_clock()->now();
        marker.ns = "target_selector";
        marker.id = 0;
        marker.type = visualization_msgs::msg::Marker::SPHERE;
        marker.action = visualization_msgs::msg::Marker::ADD;

        marker.pose.position.x = detection.bbox.center.position.x;
        marker.pose.position.y = detection.bbox.center.position.y;
        marker.pose.position.z = 0.0;
        marker.pose.orientation.w = 1.0;

        marker.scale.x = 20.0;
        marker.scale.y = 20.0;
        marker.scale.z = 1.0;

        marker.color.r = 1.0f;
        marker.color.g = 0.0f;
        marker.color.b = 0.0f;
        marker.color.a = 1.0f;

        marker.lifetime = rclcpp::Duration::from_seconds(0.5);

        marker_pub_->publish(marker);
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
