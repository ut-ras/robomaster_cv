#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "stampede_msg/msg/uart.hpp"

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
        subscription_ = this->create_subscription<stampede_msg::msg::Uart>(
            "data_tx", 10, std::bind(&UartTX::topic_callback, this, _1));
    }

  private:
    void topic_callback(const stampede_msg::msg::Uart &msg)
    {
        RCLCPP_INFO(this->get_logger(), "Frame Head Byte: %d'", msg.frame_head_byte);
        RCLCPP_INFO(this->get_logger(), "Frame Data Length: %d'", msg.frame_data_length);
        RCLCPP_INFO(this->get_logger(), "Frame Sequence: %d'", msg.frame_sequence);
        RCLCPP_INFO(this->get_logger(), "Frame CRC8: %d'", msg.frame_crc8);
        RCLCPP_INFO(this->get_logger(), "Frame MSG_TYPE: %d'", msg.msg_type);
        RCLCPP_INFO(this->get_logger(), "Data 0: %d'", msg.data[0]);
        RCLCPP_INFO(this->get_logger(), "Data 1: %d'", msg.data[1]);
        RCLCPP_INFO(this->get_logger(), "Data 2: %d'", msg.data[2]);
        RCLCPP_INFO(this->get_logger(), "CRC16: %d'", msg.crc16);

        std::vector<uint8_t> new_msg;
        new_msg.push_back(msg.frame_head_byte)
        
        int status = ser.write(new_msg);
        RCLCPP_INFO(this->get_logger(), "Status: '%d'", status);
    }
    rclcpp::Subscription<stampede_msg::msg::Uart>::SharedPtr subscription_;
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