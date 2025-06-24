#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <stampede_msgs/msg/dji_packet.hpp>
#include <stampede_msgs/msg/color_data.hpp>
#include <tf2/LinearMath/Quaternion.h>

#define MSG_TYPE_COLOR_DATA 3

class ColorNode : public rclcpp::Node {
public:
    ColorNode() : Node("color_node") {
        this->declare_parameter<std::string>("dji_rx_topic", "dji_rx");
        this->declare_parameter<std::string>("color_topic", "color_data");
        this->get_parameter("dji_rx_topic", dji_rx_topic_);
        this->get_parameter("color_topic", color_topic_);

        dji_rx_ = this->create_subscription<stampede_msgs::msg::DJIPacket>(
            dji_rx_topic_, 10,
            std::bind(&ColorNode::handle_dji_packet, this, std::placeholders::_1));
        color_pub_ = this->create_publisher<stampede_msgs::msg::ColorData>(color_topic_, 10);
    }

private:
    std::string dji_rx_topic_;
    std::string color_topic_;
    rclcpp::Subscription<stampede_msgs::msg::DJIPacket>::SharedPtr dji_rx_;
    rclcpp::Publisher<stampede_msgs::msg::ColorData>::SharedPtr color_pub_;

    void handle_dji_packet(const stampede_msgs::msg::DJIPacket::SharedPtr msg) {
        if (msg->message_type == MSG_TYPE_COLOR_DATA &&
            msg->body.size() == sizeof(stampede_msgs::msg::ColorData) - sizeof(std_msgs::msg::Header)) {
            stampede_msgs::msg::ColorData color_msg;
            color_msg.header.stamp = this->now();
            color_msg.header.frame_id = "map";
            std::memcpy(reinterpret_cast<uint8_t*>(&color_msg) + sizeof(std_msgs::msg::Header),
                        msg->body.data(),
                        msg->body.size());
            color_pub_->publish(color_msg);
            RCLCPP_INFO(this->get_logger(), "Published ColorData: (%d)", color_msg.color_type);
        }
    }
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ColorNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}