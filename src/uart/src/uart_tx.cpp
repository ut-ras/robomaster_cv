#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "stampede_msg/msg/Uart.hpp"

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
    void topic_callback(const stampede_msg::msg::Uart & msg)
    {
        RCLCPP_INFO(this->get_logger(), "Sending: '%s'", msg);
        int status = ser.write(msg);
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