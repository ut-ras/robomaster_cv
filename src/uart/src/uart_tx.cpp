#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "stampede_msgs/msg/uart.hpp"

#include "serial/serial.h"

using std::placeholders::_1;

class UartTX : public rclcpp::Node
{
  public:
    UartTX()
    : Node("uart_tx")
    {
        ser.setPort(device);
        ser.setBaudrate(115200);
        ser.open();
        int status = ser.isOpen();
        RCLCPP_INFO(this->get_logger(), "Open Device Status: '%d'", status);
        subscription_ = this->create_subscription<stampede_msgs::msg::Uart>(
            "data_tx", 10, std::bind(&UartTX::topic_callback, this, _1));
    }

  private:
    void topic_callback(const stampede_msgs::msg::Uart &msg)
    {
        RCLCPP_INFO(this->get_logger(), "Frame Head Byte: %X", msg.frame_head_byte);
        RCLCPP_INFO(this->get_logger(), "Frame Data Length: %X", msg.frame_data_length);
        RCLCPP_INFO(this->get_logger(), "Frame Sequence: %X", msg.frame_sequence);
        RCLCPP_INFO(this->get_logger(), "Frame CRC8: %X", msg.frame_crc8);
        RCLCPP_INFO(this->get_logger(), "Frame MSG_TYPE: %X", msg.msg_type);
        RCLCPP_INFO(this->get_logger(), "Data 0: %X", msg.data[0]);
        RCLCPP_INFO(this->get_logger(), "Data 1: %X", msg.data[1]);
        RCLCPP_INFO(this->get_logger(), "Data 2: %X", msg.data[2]);
        RCLCPP_INFO(this->get_logger(), "CRC16: %X", msg.crc16);

        std::vector<uint8_t> new_msg;
        new_msg.push_back(msg.frame_head_byte);
        new_msg.push_back(msg.frame_data_length & 0xFF);
        new_msg.push_back(msg.frame_data_length >> 8);
        new_msg.push_back(msg.frame_sequence);
        new_msg.push_back(msg.frame_crc8);
        new_msg.push_back(msg.msg_type & 0xFF);
        new_msg.push_back(msg.msg_type >> 8);
        new_msg.push_back(msg.data[0]);
        new_msg.push_back(msg.data[1]);
        new_msg.push_back(msg.data[2]);
        new_msg.push_back(msg.crc16 & 0xFF);
        new_msg.push_back(msg.crc16 >> 8);

        size_t status = ser.write(new_msg);
        // RCLCPP_INFO(this->get_logger(), "Status: '%lu'", status);
    }
    rclcpp::Subscription<stampede_msgs::msg::Uart>::SharedPtr subscription_;
    serial::Serial ser;
    const char* device = "/dev/ttyUSB0";
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<UartTX>());
  rclcpp::shutdown();
  return 0;
}