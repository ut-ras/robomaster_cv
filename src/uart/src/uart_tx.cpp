#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

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
        subscription_ = this->create_subscription<std_msgs::msg::String>(
            "data_tx", 10, std::bind(&UartTX::topic_callback, this, _1));
    }

  private:
    void topic_callback(const std_msgs::msg::String & msg)
    {
        RCLCPP_INFO(this->get_logger(), "Sending: '%s'", msg.data.c_str());
        int status = ser.write(msg.data.c_str());
        RCLCPP_INFO(this->get_logger(), "Status: '%d'", status);
    }
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
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