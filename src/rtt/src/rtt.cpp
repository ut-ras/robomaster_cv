#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

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

class RttNode : public rclcpp::Node
{
public:
    RttNode() : Node("rtt_node"), client_socket_(-1), retry_interval_(5.0)
    {
        this->declare_parameter<std::string>("host", "127.0.0.1");
        this->declare_parameter<int>("port", 9090);
        this->declare_parameter<double>("retry_interval", 5.0);

        this->get_parameter("host", host_);
        this->get_parameter("port", port_);
        this->get_parameter("retry_interval", retry_interval_);

        publisher_ = this->create_publisher<std_msgs::msg::String>("rtt_rx", 10);
        subscriber_ = this->create_subscription<std_msgs::msg::String>(
            "rtt_tx", 10,
            std::bind(&RttNode::handle_outgoing_message, this, std::placeholders::_1));

        connection_status_callback(true); // Initial status
        connection_thread_ = std::thread([this]() { manage_connection(); });
    }

    ~RttNode()
    {
        stop_client();
    }

private:
    std::string host_;
    int port_;
    int client_socket_;
    double retry_interval_;
    std::thread connection_thread_;
    std::thread read_thread_;

    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscriber_;

    void manage_connection()
    {
        while (rclcpp::ok())
        {
            if (client_socket_ < 0)
            {
                RCLCPP_INFO(this->get_logger(), "Attempting to connect to %s:%d...", host_.c_str(), port_);
                if (connect_to_server())
                {
                    RCLCPP_INFO(this->get_logger(), "Connected to %s:%d", host_.c_str(), port_);
                    connection_status_callback(true);
                    read_thread_ = std::thread([this]() { read_from_server(); });
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

    bool connect_to_server()
    {
        client_socket_ = socket(AF_INET, SOCK_STREAM, 0);
        if (client_socket_ < 0)
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to create socket.");
            return false;
        }

        sockaddr_in server_addr{};
        server_addr.sin_family = AF_INET;
        server_addr.sin_port = htons(port_);

        if (inet_pton(AF_INET, host_.c_str(), &server_addr.sin_addr) <= 0)
        {
            RCLCPP_ERROR(this->get_logger(), "Invalid address: %s", host_.c_str());
            return false;
        }

        if (connect(client_socket_, (struct sockaddr *)&server_addr, sizeof(server_addr)) < 0)
        {
            RCLCPP_ERROR(this->get_logger(), "Connection failed: %s", strerror(errno));
            close(client_socket_);
            client_socket_ = -1;
            return false;
        }

        return true;
    }

    void read_from_server()
    {
        char buffer[1024];
        while (rclcpp::ok())
        {
            ssize_t bytes_received = recv(client_socket_, buffer, sizeof(buffer) - 1, 0);
            if (bytes_received > 0)
            {
                buffer[bytes_received] = '\0'; // Null-terminate the received string
                std_msgs::msg::String msg;
                msg.data = std::string(buffer);
                RCLCPP_INFO(this->get_logger(), "Received: %s", msg.data.c_str());
                publisher_->publish(msg);
            }
            else if (bytes_received == 0)
            {
                RCLCPP_INFO(this->get_logger(), "Server disconnected.");
                break;
            }
            else
            {
                RCLCPP_ERROR(this->get_logger(), "Error reading from server: %s", strerror(errno));
                break;
            }
        }

        close(client_socket_);
        client_socket_ = -1;
        connection_status_callback(false); // Notify disconnection
    }

    void handle_outgoing_message(const std_msgs::msg::String::SharedPtr msg)
    {
        if (client_socket_ < 0)
        {
            RCLCPP_WARN(this->get_logger(), "Not connected to server. Cannot send message.");
            return;
        }

        std::string data = msg->data;
        ssize_t bytes_sent = send(client_socket_, data.c_str(), data.size(), 0);
        if (bytes_sent < 0)
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to send message: %s", strerror(errno));
        }
        else
        {
            RCLCPP_INFO(this->get_logger(), "Sent: %s", data.c_str());
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
        if (client_socket_ > 0)
        {
            close(client_socket_);
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
    auto node = std::make_shared<RttNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
