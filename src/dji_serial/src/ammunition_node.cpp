#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <stampede_msgs/msg/dji_packet.hpp>
#include <stampede_msgs/msg/odometry_data.hpp>
#include <cstring>

#define MSG_TYPE_HEALTH_DATA 1


class AmmunitionNode : public rclcpp::Node {
public:
    AmmunitionNode() : Node("ammunition_node") {
        this->declare_parameter<std::string>("dji_rx_topic", "dji_rx");
        this->get_parameter("dji_rx_topic", dji_rx_topic_);

        dji_rx_ = this->create_subscription<stampede_msgs::msg::DJIPacket>(
            dji_rx_topic_, 10,
            std::bind(&AmmunitionNode::handle_dji_packet, this, std::placeholders::_1));

        ammunition_pub_ = this->create_publisher<std_msgs::msg::Float32MultiArray>(ammunition_topic_, 10);

        RCLCPP_INFO(this->get_logger(), "AmmunitionNode initialized. Subscribing to '%s' and publishing to '%s'.",
                    dji_rx_topic_.c_str(), ammunition_topic_.c_str());
    }

private:
    std::string dji_rx_topic_;
    std::string ammunition_topic_ = "ammunition";
    rclcpp::Subscription<stampede_msgs::msg::DJIPacket>::SharedPtr dji_rx_;
    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr ammunition_pub_;

    void handle_dji_packet(const stampede_msgs::msg::DJIPacket::SharedPtr msg) {
        if (msg->message_type == MSG_TYPE_HEALTH_DATA &&
            msg->body.size() == sizeof(stampede_msgs::msg::AmmunitionData) - sizeof(std_msgs::msg::Header)) {

            stampede_msgs::msg::AmmunitionData ammunition_msg;
            std::memcpy(reinterpret_cast<uint8_t*>(&ammunition_msg) + sizeof(std_msgs::msg::Header),
                        msg->body.data(),
                        msg->body.size());

            // Construct ammunition data array
            std_msgs::msg::Float32MultiArray ammunition_array;
            ammunition_array.data = { ammunition_msg.ammunition_left };
            ammunition_pub_->publish(ammunition_array);

            RCLCPP_INFO(this->get_logger(), "Published Ammunition: %.2f", ammunition_msg.ammunition_left);
        } else {
            RCLCPP_WARN(this->get_logger(),
                        "Received unexpected DJI packet type (%d) or size (%zu)",
                        msg->message_type, msg->body.size());
        }
    }
};



int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<AmmunitionNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
