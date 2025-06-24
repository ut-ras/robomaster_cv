#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/byte_multi_array.hpp"
#include "stampede_msgs/msg/dji_packet.hpp"
#include "stampede_msgs/msg/turret_data.hpp"

using namespace std::chrono_literals;

class TurretSendNode : public rclcpp::Node
{
public:
    TurretSendNode()
    : Node("turret_send_node")
    {
        // is this the correct publisher name? 
        publisher_ = this->create_publisher<stampede_msgs::msg::DJIPacket>("dji_tx", 10);
        subscription_ = this->create_subscription<stampede_msgs::msg::TurretData>(
            "turret_data", 10, std::bind(&TestSerialNode::topic_callback, this, std::placeholders::_1));
    }

private:
    

    void topic_callback(const stampede_msgs::msg::TurretData::SharedPtr msg)
    {
        auto message = stampede_msgs::msg::DJIPacket();
        message.frame_head_byte = 0xA5;
        message.frame_data_length = sizeof(stampede_msgs::msg::TurretData);
        // message.frame_sequence_number = count_++;
        message.frame_sequence_number = 0; 
        message.crc8 = 0x00; // Placeholder CRC8
        message.message_type = 0x0002;

        // uint8_t body_bytes[sizeof(stampede_msgs::msg::TurretData)];
        // body_bytes[0] = msg->xpos;
        // body_bytes[4] = msg->ypos; 

        size_t total_size = sizeof(stampede_msgs::msg::TurretData);
        
        std::vector<uint8_t> buffer(total_size);
        std::uint8_t* buffer_ptr = buffer.data();

        // starting position, data, size 
        std::memcpy(buffer_ptr, &msg->xpos, sizeof(_Float32));
        buffer_ptr += sizeof(_Float32);  // increase starting position

        std::memcpy(buffer_ptr, &msg->ypos, sizeof(_Float32));
        buffer_ptr += sizeof(_Float32);  

        std::memcpy(buffer_ptr, &msg->zpos, sizeof(_Float32));
        buffer_ptr += sizeof(_Float32);  

        std::memcpy(buffer_ptr, &msg->xvel, sizeof(_Float32));
        buffer_ptr += sizeof(_Float32);  

        std::memcpy(buffer_ptr, &msg->yvel, sizeof(_Float32));
        buffer_ptr += sizeof(_Float32);  

        std::memcpy(buffer_ptr, &msg->zvel, sizeof(_Float32));
        buffer_ptr += sizeof(_Float32);  

        std::memcpy(buffer_ptr, &msg->xacc, sizeof(_Float32));
        buffer_ptr += sizeof(_Float32);  

        std::memcpy(buffer_ptr, &msg->yacc, sizeof(_Float32));
        buffer_ptr += sizeof(_Float32);  

        std::memcpy(buffer_ptr, &msg->zacc, sizeof(_Float32));
        buffer_ptr += sizeof(_Float32); 
        
        std::memcpy(buffer_ptr, &msg->has_target, sizeof(bool));
        buffer_ptr += sizeof(bool);  

        message.body = buffer;

        message.crc16 = 0x0000; // Placeholder CRC16

        publisher_->publish(message);
    }

    rclcpp::Publisher<stampede_msgs::msg::DJIPacket>::SharedPtr publisher_;
    rclcpp::Subscription<stampede_msgs::msg::TurretData>::SharedPtr subscription_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TurretSendNode>());
    rclcpp::shutdown();
    return 0;
}