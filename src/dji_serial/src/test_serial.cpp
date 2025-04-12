#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/byte_multi_array.hpp"
#include "stampede_msgs/msg/dji_packet.hpp"

using namespace std::chrono_literals;

class TestSerialNode : public rclcpp::Node
{
public:
    TestSerialNode()
    : Node("test_serial_node")
    {
        publisher_ = this->create_publisher<stampede_msgs::msg::DJIPacket>("dji_tx", 10);
        subscription_ = this->create_subscription<stampede_msgs::msg::DJIPacket>(
            "dji_rx", 10, std::bind(&TestSerialNode::topic_callback, this, std::placeholders::_1));

        timer_ = this->create_wall_timer(
            1000ms, std::bind(&TestSerialNode::timer_callback, this));
    }

private:
    void timer_callback()
    {
        auto message = stampede_msgs::msg::DJIPacket();
        message.frame_head_byte = 0xA5;
        message.frame_data_length = 32;
        // message.frame_sequence_number = count_++;
        message.frame_sequence_number = 0; 
        message.crc8 = 0x00; // Placeholder CRC8
        message.message_type = 0x0001;
        message.body = {0x04, 0x02, 0x03};
        message.body = {0x10, 0x01, 0x00, 0x00, 0x00, 0xe3, 0x65, 0x03, 0x08, 0xb1, 0x78,
            0x00, 0x08, 0xb5, 0x08, 0x00, 0x08, 0xbd, 0x08, 0x00, 0x08, 0xb5, 0x08, 0x00,
            0x08, 0xa5, 0x07, 0x00, 0x08, 0x95, 0x07, 0x00};
        message.crc16 = 0x0000; // Placeholder CRC16

        RCLCPP_INFO(this->get_logger(), "Publishing DJI Packet: seq=%d", message.frame_sequence_number);
        publisher_->publish(message);
    }

    void topic_callback(const stampede_msgs::msg::DJIPacket::SharedPtr msg)
    {
        std::string hex_data;
        for (auto byte : msg->body)
        {
            char hex[3];
            snprintf(hex, sizeof(hex), "%02X", static_cast<unsigned char>(byte));
            hex_data += hex;
        }
        RCLCPP_INFO(this->get_logger(), "Received DJI Packet: seq=%d, type=%d size=%ld body=%s", msg->frame_sequence_number, msg->message_type, msg->body.size(), hex_data.c_str());
    }

    rclcpp::Publisher<stampede_msgs::msg::DJIPacket>::SharedPtr publisher_;
    rclcpp::Subscription<stampede_msgs::msg::DJIPacket>::SharedPtr subscription_;
    rclcpp::TimerBase::SharedPtr timer_;
    size_t count_ = 0;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TestSerialNode>());
    rclcpp::shutdown();
    return 0;
}