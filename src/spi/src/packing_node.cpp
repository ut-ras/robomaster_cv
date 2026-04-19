#include <bit>
#include <cstdint>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/byte_multi_array.hpp>
#include <stampede_msgs/msg/spi_health_data.hpp>
#include <stampede_msgs/msg/spi_odometry_data.hpp>
#include <stampede_msgs/msg/spi_position_data.hpp>
#include <stampede_msgs/msg/spi_enemy_data.hpp>
#include <cstring>

using std::placeholders::_1;


class PackingNode : public rclcpp::Node
{
public:
    PackingNode() : Node("packing_node")
    {
        RCLCPP_INFO(this->get_logger(), "Packing node started.");
        loc_sub = this->create_subscription<stampede_msgs::msg::SpiOdometryData>(
            "loc_data", 10, std::bind(&PackingNode::loc_callback, this, std::placeholders::_1));

        path_sub = this->create_subscription<stampede_msgs::msg::SpiPositionData>(
            "path_data", 10, std::bind(&PackingNode::path_callback, this, std::placeholders::_1));
        
        enemy_sub = this->create_subscription<stampede_msgs::msg::SpiEnemyData>(
            "enemy_data", 10, std::bind(&PackingNode::enemy_callback, this, std::placeholders::_1));

        timer_ = this->create_wall_timer(std::chrono::milliseconds(1000/30), std::bind(&PackingNode::callback, this));

        pack_pub_ = this->create_publisher<std_msgs::msg::ByteMultiArray>("spi_tx", 10);

        odom_data = std_msgs::msg::ByteMultiArray();
        path_data = std_msgs::msg::ByteMultiArray();
        enemy_data = std_msgs::msg::ByteMultiArray();
    }

private:
    void loc_callback(const stampede_msgs::msg::SpiOdometryData::SharedPtr msg)
    {
        unsigned char x_pos_bytes[sizeof(msg->x_pos)];
        unsigned char y_pos_bytes[sizeof(msg->y_pos)];
        std::memcpy(x_pos_bytes, &msg->x_pos, sizeof(msg->x_pos));
        std::memcpy(y_pos_bytes, &msg->y_pos, sizeof(msg->y_pos));
        odom_data.data = {x_pos_bytes[0], x_pos_bytes[1], x_pos_bytes[2], x_pos_bytes[3],
                          y_pos_bytes[0], y_pos_bytes[1], y_pos_bytes[2], y_pos_bytes[3]};
    }

    void path_callback(const stampede_msgs::msg::SpiPositionData::SharedPtr msg){
        unsigned char x_pos_bytes[sizeof(msg->x_pos)];
        unsigned char y_pos_bytes[sizeof(msg->y_pos)];
        std::memcpy(x_pos_bytes, &msg->x_pos, sizeof(msg->x_pos));
        std::memcpy(y_pos_bytes, &msg->y_pos, sizeof(msg->y_pos));
        path_data.data = {x_pos_bytes[0], x_pos_bytes[1], x_pos_bytes[2], x_pos_bytes[3],
                          y_pos_bytes[0], y_pos_bytes[1], y_pos_bytes[2], y_pos_bytes[3]};
    }

    void enemy_callback(const stampede_msgs::msg::SpiEnemyData::SharedPtr msg){
        unsigned char x_pos_bytes[sizeof(msg->x_pos)];
        unsigned char y_pos_bytes[sizeof(msg->y_pos)];
        unsigned char z_pos_bytes[sizeof(msg->z_pos)];
        unsigned char real_target_bytes[sizeof(msg->real_target)];
        std::memcpy(x_pos_bytes, &msg->x_pos, sizeof(msg->x_pos));
        std::memcpy(y_pos_bytes, &msg->y_pos, sizeof(msg->y_pos));
        std::memcpy(z_pos_bytes, &msg->z_pos, sizeof(msg->z_pos));
        std::memcpy(real_target_bytes, &msg->real_target, sizeof(msg->real_target));
        enemy_data.data = {x_pos_bytes[0], x_pos_bytes[1], x_pos_bytes[2], x_pos_bytes[3],
                           y_pos_bytes[0], y_pos_bytes[1], y_pos_bytes[2], y_pos_bytes[3],
                           z_pos_bytes[0], z_pos_bytes[1], z_pos_bytes[2], z_pos_bytes[3],
                           real_target_bytes[0]};
    }

    void callback()
    {
        std_msgs::msg::ByteMultiArray msg;
        msg.data.insert(msg.data.end(), odom_data.data.begin(), odom_data.data.end());
        msg.data.insert(msg.data.end(), path_data.data.begin(), path_data.data.end());
        msg.data.insert(msg.data.end(), enemy_data.data.begin(), enemy_data.data.end());
        pack_pub_->publish(msg);
    }


    rclcpp::Subscription<stampede_msgs::msg::SpiOdometryData>::SharedPtr loc_sub;
    rclcpp::Subscription<stampede_msgs::msg::SpiPositionData>::SharedPtr path_sub;
    rclcpp::Subscription<stampede_msgs::msg::SpiEnemyData>::SharedPtr enemy_sub;
    rclcpp::Publisher<std_msgs::msg::ByteMultiArray>::SharedPtr pack_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
    
    std_msgs::msg::ByteMultiArray odom_data;
    std_msgs::msg::ByteMultiArray path_data;
    std_msgs::msg::ByteMultiArray enemy_data;
};




int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PackingNode>());
    rclcpp::shutdown();
    return 0;
}