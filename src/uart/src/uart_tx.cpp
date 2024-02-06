#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

#include "uart/serialib.h"

using std::placeholders::_1;

class UartTX : public rclcpp::Node
{
  public:
    UartTX()
    : Node("uart_tx")
    {
        serial.openDevice(device, 115200);
        subscription_ = this->create_subscription<std_msgs::msg::String>(
            "data_tx", 10, std::bind(&UartTX::topic_callback, this, _1));
    }

  private:
    void topic_callback(const std_msgs::msg::String & msg)
    {
        RCLCPP_INFO(this->get_logger(), "Sending: '%s'", msg.data.c_str());
        serial.writeString(msg.data.c_str());
    }
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
    serialib serial;
    const char* device = "/dev/ttyUSB0";
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<UartTX>());
  rclcpp::shutdown();
  return 0;
}