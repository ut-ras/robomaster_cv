#include <bit>
#include <cstdint>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/byte_multi_array.hpp>
#include <stampede_msgs/msg/spi_health_data.hpp>
#include <stampede_msgs/msg/spi_odometry_data.hpp>
#include <cstring>

using std::placeholders::_1;


class UnpackingNode : public rclcpp::Node
{
public:
    UnpackingNode() : Node("unpacking_node")
    {
        RCLCPP_INFO(this->get_logger(), "Unpacking node started.");
        subscription_ = this->create_subscription<std_msgs::msg::ByteMultiArray>(
            "spi_rx", 10, std::bind(&UnpackingNode::topic_callback, this, std::placeholders::_1));
        odom_pub_ = this->create_publisher<stampede_msgs::msg::SpiOdometryData>("odom_data", 10);
        health_pub_ = this->create_publisher<stampede_msgs::msg::SpiHealthData>("health_data", 10);
    }

private:
    void topic_callback(const std_msgs::msg::ByteMultiArray::SharedPtr msg)
    {
        stampede_msgs::msg::SpiOdometryData odom_data;
        // odom_data.x_pos = std::bit_cast<float>({msg.data[0], msg.data[1], msg.data[2], msg.data[3]});
        // odom_data.y_pos = std::bit_cast<float>({msg.data[4], msg.data[5], msg.data[6], msg.data[7]});

        uint8_t odom_x_pos_bytes[4] = {msg->data[0], msg->data[1], msg->data[2], msg->data[3]};
        uint8_t odom_y_pos_bytes[4] = {msg->data[4], msg->data[5], msg->data[6], msg->data[7]};
        float odom_x_pos, odom_y_pos;
        std::memcpy(&odom_x_pos, odom_x_pos_bytes, sizeof(odom_x_pos));
        std::memcpy(&odom_y_pos, odom_y_pos_bytes, sizeof(odom_y_pos));
        odom_data.x_pos = odom_x_pos;
        odom_data.y_pos = odom_y_pos;

        stampede_msgs::msg::SpiHealthData health_data;        
        // health_data.health = std::bit_cast<float>({msg.data[8], msg.data[9], msg.data[10], msg.data[11]});
        // health_data.color = msg.data[12] != 0;
        uint8_t health_health_bytes[4] = {msg->data[8], msg->data[9], msg->data[10], msg->data[11]};
        float health_health;
        std::memcpy(&health_health, health_health_bytes, sizeof(health_health));
        health_data.health = health_health;
        health_data.color = msg->data[12] != 0;


        odom_pub_->publish(odom_data);
        health_pub_->publish(health_data);
    }

    rclcpp::Subscription<std_msgs::msg::ByteMultiArray>::SharedPtr subscription_;
    rclcpp::Publisher<stampede_msgs::msg::SpiOdometryData>::SharedPtr odom_pub_;
    rclcpp::Publisher<stampede_msgs::msg::SpiHealthData>::SharedPtr health_pub_;
};


int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<UnpackingNode>());
    rclcpp::shutdown();
    return 0;
}