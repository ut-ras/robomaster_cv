#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <stampede_msgs/msg/dji_packet.hpp>
#include <stampede_msgs/msg/odometry_data.hpp>
#include <tf2/LinearMath/Quaternion.h>

#define MSG_TYPE_ODOMETRY_DATA 1

class PoseNode : public rclcpp::Node {
public:
    PoseNode() : Node("pose_node") {
        this->declare_parameter<std::string>("dji_rx_topic", "dji_tx");
        this->declare_parameter<std::string>("odometry_topic", "odometry_data");
        this->get_parameter("dji_rx_topic", dji_rx_topic_);
        this->get_parameter("odometry_topic", odometry_topic_);

        dji_rx_ = this->create_subscription<stampede_msgs::msg::DJIPacket>(
            dji_rx_topic_, 10,
            std::bind(&PoseNode::handle_dji_packet, this, std::placeholders::_1));
        odom_pub_ = this->create_publisher<stampede_msgs::msg::OdometryData>(odometry_topic_, 10);
    }

private:
    std::string dji_rx_topic_;
    std::string odometry_topic_;
    rclcpp::Subscription<stampede_msgs::msg::DJIPacket>::SharedPtr dji_rx_;
    rclcpp::Publisher<stampede_msgs::msg::OdometryData>::SharedPtr odom_pub_;

    void handle_dji_packet(const stampede_msgs::msg::DJIPacket::SharedPtr msg) {
        if (msg->message_type == MSG_TYPE_ODOMETRY_DATA &&
            msg->body.size() == sizeof(stampede_msgs::msg::OdometryData) - sizeof(std_msgs::msg::Header)) {
            stampede_msgs::msg::OdometryData odom_msg;
            odom_msg.header.stamp = this->now();
            odom_msg.header.frame_id = "map";
            std::memcpy(reinterpret_cast<uint8_t*>(&odom_msg) + sizeof(std_msgs::msg::Header),
                        msg->body.data(),
                        msg->body.size());
            odom_pub_->publish(odom_msg);
            RCLCPP_INFO(this->get_logger(), "Published OdometryData: (%.2f, %.2f, yaw=%.2f)", odom_msg.x_pos, odom_msg.y_pos, odom_msg.chassis_yaw);
        }
    }
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<PoseNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}