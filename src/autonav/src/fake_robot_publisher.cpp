#include <rclcpp/rclcpp.hpp>
#include <vision_msgs/msg/detection3_d_array.hpp>
#include <vision_msgs/msg/detection3_d.hpp>
#include <chrono>

class FakeRobotPublisher : public rclcpp::Node {
public:
    FakeRobotPublisher() : Node("fake_robot_publisher") {
        publisher_ = this->create_publisher<vision_msgs::msg::Detection3DArray>("robots_topic", 10);
        
        // Publish fake robot positions every 1 second
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(1000),
            std::bind(&FakeRobotPublisher::publish_fake_robots, this));
        
        RCLCPP_INFO(this->get_logger(), "Fake robot publisher started. Publishing to 'robots_topic'");
    }

private:
    void publish_fake_robots() {
        auto msg = vision_msgs::msg::Detection3DArray();
        
        // Define fake robot positions for testing
        // These positions are in meters relative to field center
        // Field is typically 12m x 12m, so positions are in range [-6, 6]
        std::vector<std::vector<float>> fake_positions = {
            {2.0f, 1.5f, 0.0f},   // Robot 1: near capture point
            {-3.0f, -2.0f, 0.0f},  // Robot 2: far from capture point
            {0.5f, 0.5f, 0.0f}     // Robot 3: very close to capture point
        };
        
        for (const auto& pos : fake_positions) {
            vision_msgs::msg::Detection3D detection;
            detection.bbox.center.position.x = pos[0];
            detection.bbox.center.position.y = pos[1];
            detection.bbox.center.position.z = pos[2];
            msg.detections.push_back(detection);
        }
        
        publisher_->publish(msg);
        RCLCPP_INFO(this->get_logger(), "Published %zu fake robot positions", msg.detections.size());
    }
    
    rclcpp::Publisher<vision_msgs::msg::Detection3DArray>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<FakeRobotPublisher>());
    rclcpp::shutdown();
    return 0;
}

