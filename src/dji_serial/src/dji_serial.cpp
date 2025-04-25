#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/byte_multi_array.hpp>
#include <stampede_msgs/msg/dji_packet.hpp>
#include "dji_serial_packet.hpp"
#include "crc.hpp" 
#include "pose_packet.hpp"
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <tf2/LinearMath/Quaternion.h>

class DJISerialNode : public rclcpp::Node
{
public:
    DJISerialNode() : Node("dji_serial_node")
    {
        this->declare_parameter<std::string>("serial_tx_topic", "uart_tx");
        this->declare_parameter<std::string>("serial_rx_topic", "uart_rx");
        this->declare_parameter<std::string>("dji_tx_topic", "dji_rx");
        this->declare_parameter<std::string>("dji_rx_topic", "dji_tx");
        this->get_parameter("serial_tx_topic", serial_tx_topic_);
        this->get_parameter("serial_rx_topic", serial_rx_topic_);
        this->get_parameter("dji_tx_topic", dji_tx_topic_);
        this->get_parameter("dji_rx_topic", dji_rx_topic_);

        rtt_tx_ = this->create_publisher<std_msgs::msg::ByteMultiArray>(serial_tx_topic_, 10);
        rtt_rx_ = this->create_subscription<std_msgs::msg::ByteMultiArray>(
            serial_rx_topic_, 10,
            std::bind(&DJISerialNode::handle_rtt_message, this, std::placeholders::_1));
        dji_tx_ = this->create_publisher<stampede_msgs::msg::DJIPacket>(dji_tx_topic_, 10);
        dji_rx_ = this->create_subscription<stampede_msgs::msg::DJIPacket>(
            dji_rx_topic_, 10,
            std::bind(&DJISerialNode::handle_dji_packet, this, std::placeholders::_1));

        pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("sentry/pose", 10);
    }

private:
    std::string serial_tx_topic_;
    std::string serial_rx_topic_;
    std::string dji_tx_topic_;
    std::string dji_rx_topic_;
    rclcpp::Publisher<std_msgs::msg::ByteMultiArray>::SharedPtr rtt_tx_;
    rclcpp::Subscription<std_msgs::msg::ByteMultiArray>::SharedPtr rtt_rx_;
    rclcpp::Publisher<stampede_msgs::msg::DJIPacket>::SharedPtr dji_tx_;
    rclcpp::Subscription<stampede_msgs::msg::DJIPacket>::SharedPtr dji_rx_;

    std::vector<uint8_t> buffer_;

    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;

    void handle_dji_packet(const stampede_msgs::msg::DJIPacket::SharedPtr msg)
    {
        RCLCPP_INFO(this->get_logger(), "Received DJI Packet: seq=%d, type=%d", msg->frame_sequence_number, msg->message_type);
        uint8_t body_bytes[msg->body.size()];
        std::copy(msg->body.begin(), msg->body.end(), body_bytes);
        
        SerialMessage serial_msg(body_bytes, msg->body.size(), msg->message_type, msg->frame_sequence_number);
        std_msgs::msg::ByteMultiArray dji_packet;

        dji_packet.data.insert(dji_packet.data.end(), serial_msg.buffer, serial_msg.buffer + serial_msg.length);
        rtt_tx_->publish(dji_packet);
        std::string hex_data;
        for (auto byte : dji_packet.data)
        {
            char hex[3];
            snprintf(hex, sizeof(hex), "%02x", byte);
            hex_data += hex;
        }
        RCLCPP_INFO(this->get_logger(), "Published %s", hex_data.c_str());
    }

    void handle_rtt_message(const std_msgs::msg::ByteMultiArray::SharedPtr msg)
    {
        RCLCPP_INFO(this->get_logger(), "Received RTT message");

        // Append incoming data to buffer
        buffer_.insert(buffer_.end(), msg->data.begin(), msg->data.end());

        while (buffer_.size() >= 9) // Minimum size check
        {
            // Search for the start of a packet
            auto it = std::find(buffer_.begin(), buffer_.end(), SERIAL_HEAD_BYTE);
            if (it == buffer_.end())
            {
                // No start byte found, clear buffer
                buffer_.clear();
                return;
            }

            // Remove data before the start byte
            buffer_.erase(buffer_.begin(), it);

            if (buffer_.size() < 9)
            {
                // Not enough data for a complete header
                return;
            }

            // Parse the packet header
            auto dji_packet = stampede_msgs::msg::DJIPacket();
            dji_packet.frame_head_byte = buffer_[0];
            dji_packet.frame_data_length = static_cast<uint16_t>(buffer_[1]) | (static_cast<uint16_t>(buffer_[2]) << 8);
            dji_packet.frame_sequence_number = buffer_[3];
            dji_packet.crc8 = buffer_[4];
            dji_packet.message_type = static_cast<uint16_t>(buffer_[5]) | (static_cast<uint16_t>(buffer_[6]) << 8);

            size_t packet_size = 7 + dji_packet.frame_data_length + 2; // Header + data length + CRC16

            if (buffer_.size() < packet_size)
            {
                // Not enough data for a complete packet
                return;
            }

            // Parse the packet body and CRC16
            dji_packet.body.insert(dji_packet.body.end(), buffer_.begin() + 7, buffer_.begin() + 7 + dji_packet.frame_data_length);
            dji_packet.crc16 = static_cast<uint16_t>(buffer_[7 + dji_packet.frame_data_length]) | (static_cast<uint16_t>(buffer_[8 + dji_packet.frame_data_length]) << 8);

            // Verify CRC
            uint8_t crc8 = algorithms::calculateCRC8(buffer_.data(), 4);
            if (crc8 != dji_packet.crc8)
            {
                RCLCPP_ERROR(this->get_logger(), "CRC8 mismatch");
                buffer_.erase(buffer_.begin(), buffer_.begin() + packet_size);
                continue;
            }
            uint16_t crc16 = algorithms::calculateCRC16(buffer_.data(), packet_size - 2);
            if (crc16 != dji_packet.crc16)
            {
                RCLCPP_ERROR(this->get_logger(), "CRC16 mismatch");
                buffer_.erase(buffer_.begin(), buffer_.begin() + packet_size);
                continue;
            }

            #define MSG_TYPE_ODOMETRY_DATA 1

            if (dji_packet.message_type == MSG_TYPE_ODOMETRY_DATA && dji_packet.body.size() == sizeof(OdometryData))
            {
                OdometryData data;
                std::memcpy(&data, dji_packet.body.data(), sizeof(OdometryData));

                geometry_msgs::msg::PoseStamped pose_msg;
                pose_msg.header.stamp = this->now();
                pose_msg.header.frame_id = "map";

                pose_msg.pose.position.x = data.xPos;
                pose_msg.pose.position.y = data.yPos;
                pose_msg.pose.position.z = 0.0;

                tf2::Quaternion q;
                q.setRPY(0, 0, data.chassisYaw);
                pose_msg.pose.orientation.x = q.x();
                pose_msg.pose.orientation.y = q.y();
                pose_msg.pose.orientation.z = q.z();
                pose_msg.pose.orientation.w = q.w();

                pose_pub_->publish(pose_msg);
                RCLCPP_INFO(this->get_logger(), "Published Pose: (%.2f, %.2f, yaw=%.2f)", data.xPos, data.yPos, data.chassisYaw);
            }
            // Publish the packet
            dji_tx_->publish(dji_packet);

            // Remove the processed packet from the buffer
            buffer_.erase(buffer_.begin(), buffer_.begin() + packet_size);
        }
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<DJISerialNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}