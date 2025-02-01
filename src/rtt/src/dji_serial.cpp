#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/byte_multi_array.hpp>
#include <stampede_msgs/msg/dji_packet.hpp>
#include "dji_serial_packet.hpp"
#include "crc.hpp"

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

    void handle_dji_packet(const stampede_msgs::msg::DJIPacket::SharedPtr msg)
    {
        uint8_t body_bytes[msg->body.size()];
        std::copy(msg->body.begin(), msg->body.end(), body_bytes);
        
        SerialMessage serial_msg(body_bytes, msg->body.size(), msg->frame_sequence_number);
        std_msgs::msg::ByteMultiArray dji_packet;

        dji_packet.data.insert(dji_packet.data.end(), serial_msg.buffer, serial_msg.buffer + serial_msg.length);
        rtt_tx_->publish(dji_packet);
    }

    void handle_rtt_message(const std_msgs::msg::ByteMultiArray::SharedPtr msg)
    {
        if (msg->data.size() < 9) // Minimum size check
        {
            RCLCPP_ERROR(this->get_logger(), "Received invalid DJI message");
            return;
        }

        auto dji_packet = stampede_msgs::msg::DJIPacket();
        dji_packet.frame_head_byte = msg->data[0];
        dji_packet.frame_data_length = static_cast<uint16_t>(msg->data[1]) | (static_cast<uint16_t>(msg->data[2]) << 8);
        dji_packet.frame_sequence_number = msg->data[3];
        dji_packet.crc8 = msg->data[4];
        dji_packet.message_type = static_cast<uint16_t>(msg->data[5]) | (static_cast<uint16_t>(msg->data[6]) << 8);
        dji_packet.body.insert(dji_packet.body.end(), msg->data.begin() + 7, msg->data.end() - 2);
        dji_packet.crc16 = static_cast<uint16_t>(*(msg->data.end() - 2)) | (static_cast<uint16_t>(*(msg->data.end() - 1)) << 8);

        // verify crc
        uint8_t crc8 = algorithms::calculateCRC8(msg->data.data(), 4);
        if (crc8 != dji_packet.crc8)
        {
            RCLCPP_ERROR(this->get_logger(), "CRC8 mismatch");
            return;
        }
        uint16_t crc16 = algorithms::calculateCRC16(msg->data.data(), msg->data.size() - 2);
        if (crc16 != dji_packet.crc16)
        {
            RCLCPP_ERROR(this->get_logger(), "CRC16 mismatch");
            return;
        }

        dji_tx_->publish(dji_packet);
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