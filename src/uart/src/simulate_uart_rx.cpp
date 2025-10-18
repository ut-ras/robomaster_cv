#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/byte_multi_array.hpp>
#include <string>
#include <vector>
#include <sstream>
#include <iomanip>
#include <algorithm>

// Helper: Convert hex string (e.g. "01A3FF") to std::vector<uint8_t>
std::vector<uint8_t> hex_string_to_bytes(const std::string &hex) {
    std::vector<uint8_t> bytes;
    std::string clean_hex;
    // Remove spaces
    std::remove_copy_if(hex.begin(), hex.end(), std::back_inserter(clean_hex), ::isspace);
    if (clean_hex.length() % 2 != 0) return bytes; // Invalid length

    for (size_t i = 0; i < clean_hex.length(); i += 2) {
        std::string byte_str = clean_hex.substr(i, 2);
        uint8_t byte = static_cast<uint8_t>(std::stoul(byte_str, nullptr, 16));
        bytes.push_back(byte);
    }
    return bytes;
}

class SimulateUartRxNode : public rclcpp::Node {
public:
    SimulateUartRxNode() : Node("simulate_uart_rx") {
        publisher_ = this->create_publisher<std_msgs::msg::ByteMultiArray>("uart_rx", 10);

        this->declare_parameter<std::string>("hex_bytes", "");
        this->declare_parameter<double>("publish_rate", 1.0); // Default 1 Hz
        std::string hex_bytes;
        double publish_rate;
        this->get_parameter("hex_bytes", hex_bytes);
        this->get_parameter("publish_rate", publish_rate);

        if (hex_bytes.empty()) {
            RCLCPP_ERROR(this->get_logger(), "No hex_bytes parameter provided.");
            rclcpp::shutdown();
            return;
        }

        bytes_ = hex_string_to_bytes(hex_bytes);
        if (bytes_.empty()) {
            RCLCPP_ERROR(this->get_logger(), "Failed to parse hex_bytes: '%s'", hex_bytes.c_str());
            rclcpp::shutdown();
            return;
        }

        timer_ = this->create_wall_timer(
            std::chrono::duration<double>(1.0 / publish_rate),
            std::bind(&SimulateUartRxNode::timer_callback, this)
        );
    }

private:
    void timer_callback() {
        std_msgs::msg::ByteMultiArray msg;
        msg.data = bytes_;
        std::ostringstream oss;
        for (auto b : bytes_) oss << std::hex << std::setw(2) << std::setfill('0') << (int)b;
        RCLCPP_INFO(this->get_logger(), "Publishing %zu bytes: %s", bytes_.size(), oss.str().c_str());
        publisher_->publish(msg);
    }

    rclcpp::Publisher<std_msgs::msg::ByteMultiArray>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    std::vector<uint8_t> bytes_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<SimulateUartRxNode>();
    rclcpp::spin(node);
    return 0;
}