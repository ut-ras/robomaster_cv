#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <serial/serial.h>

#include <iostream>
#include <memory>
#include <string>
#include <thread>
#include <vector>
#include <chrono>

#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>

class UartNode : public rclcpp::Node
{
public:
    UartNode() : Node("uart_node"), retry_interval_(5.0)
    {
        this->declare_parameter<std::string>("port", "/dev/ttyUSB0");
        this->declare_parameter<double>("retry_interval", 5.0);

        this->get_parameter("port", port_);
        this->get_parameter("retry_interval", retry_interval_);

        publisher_ = this->create_publisher<std_msgs::msg::String>("uart_rx", 10);
        subscriber_ = this->create_subscription<std_msgs::msg::String>(
            "uart_tx", 10,
            std::bind(&UartNode::handle_outgoing_message, this, std::placeholders::_1));

        connection_status_callback(true); // Initial status
        connection_thread_ = std::thread([this]() { manage_connection(); });
    }

    ~UartNode()
    {
        stop_client();
    }

private:
    std::string port_;
    serial::Serial serial_;
    double retry_interval_;
    std::thread connection_thread_;
    std::thread read_thread_;

    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscriber_;

    void manage_connection()
    {
        while (rclcpp::ok())
        {
            if (!serial_.isOpen())
            {
                RCLCPP_INFO(this->get_logger(), "Attempting to connect to %s...", port_.c_str());
                if (connect_to_port())
                {
                    RCLCPP_INFO(this->get_logger(), "Connected to %s", port_.c_str());
                    connection_status_callback(true);
                    read_thread_ = std::thread([this]() { read_from_port(); });
                }
                else
                {
                    RCLCPP_WARN(this->get_logger(), "Connection attempt failed. Retrying in %.2f seconds.", retry_interval_);
                    connection_status_callback(false);
                }
            }
            std::this_thread::sleep_for(std::chrono::duration<double>(retry_interval_));
        }
    }

    bool connect_to_port()
    {
        if (serial_.isOpen()) {
            // Log warning if port is already open
            RCLCPP_WARN(this->get_logger(), "Port is already open. Closing...");
            serial_.close();
        }

        serial_.setPort(port_);
        serial_.setBaudrate(115200);

        try
        {
            serial_.open();
        }
        catch(const std::invalid_argument& e)
        {
            RCLCPP_ERROR(this->get_logger(), "Invalid argument when connecting to %s: %s", port_.c_str(), e.what());
            return false;
        }
        catch (const serial::IOException& e)
        {
            RCLCPP_ERROR(this->get_logger(), "IO Exception when connecting to %s: %s", port_.c_str(), e.what());
            return false;
        }
        catch (const serial::SerialException& e)
        {
            RCLCPP_ERROR(this->get_logger(), "Serial Exception when connecting to %s: %s", port_.c_str(), e.what());
            return false;
        }

        return serial_.isOpen();
    }

    void read_from_port()
    {
        char buffer[1024];
        while (rclcpp::ok())
        {
            if (serial_.available()) 
            {
                std::string data = serial_.read(serial_.available());
                RCLCPP_INFO(this->get_logger(), "Received: '%s'", data.c_str());
                auto message = std_msgs::msg::String();
                message.data = data;
                publisher_->publish(message);
            }

            // Better version of code above
            std::string data = serial_.read(serial_.available());
            if (!data.empty())
            {
                RCLCPP_INFO(this->get_logger(), "Received: '%s'", data.c_str());
                auto message = std_msgs::msg::String();
                message.data = data;
                publisher_->publish(message);
            }
        }

        serial_.close();
        connection_status_callback(false); // Notify disconnection
    }

    void handle_outgoing_message(const std_msgs::msg::String::SharedPtr msg)
    {
        if (serial_.isOpen())
        {
            serial_.write(msg->data);
            RCLCPP_INFO(this->get_logger(), "Sent: %s", msg->data.c_str());
        }
        else
        {
            RCLCPP_WARN(this->get_logger(), "Not connected to port. Cannot send message.");
        }
    }

    void connection_status_callback(bool connected)
    {
        // TODO: Update status
        if (connected)
        {
            RCLCPP_INFO(this->get_logger(), "Connection status: CONNECTED");
        }
        else
        {
            RCLCPP_WARN(this->get_logger(), "Connection status: DISCONNECTED");
        }
    }

    void stop_client()
    {
        if (serial_.isOpen())
        {
            serial_.close();
        }

        if (read_thread_.joinable())
        {
            read_thread_.join();
        }

        if (connection_thread_.joinable())
        {
            connection_thread_.join();
        }
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<UartNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
