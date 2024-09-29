#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

#include "serial/serial.h"


using namespace std::chrono_literals;

/* This example creates a subclass of Node and uses std::bind() to register a
* member function as a callback from the timer. */

class UartRX : public rclcpp::Node
{
  public:
    UartRX()
    : Node("uart_rx")
    {
        ser.setPort(device);
        ser.setBaudrate(115200);
        if (!ser.isOpen()) {
            ser.open();
        }
        int status = ser.isOpen();
        RCLCPP_INFO(this->get_logger(), "Open Device Status: '%d'", status);
        
        publisher_ = this->create_publisher<std_msgs::msg::String>("data_rx", 10);
        timer_ = this->create_wall_timer(
        5ms, std::bind(&UartRX::timer_callback, this));
    }

  private:
    void timer_callback()
    {
        if (ser.available()) 
        {
            std::string data = ser.read(ser.available());
            RCLCPP_INFO(this->get_logger(), "Received: '%s'", data.c_str());
            auto message = std_msgs::msg::String();
            message.data = data;
            publisher_->publish(message);
        }
    }
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
    serial::Serial ser;
    const char* device = "/dev/ttyUSB0";
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<UartRX>());
  rclcpp::shutdown();
  return 0;
}
