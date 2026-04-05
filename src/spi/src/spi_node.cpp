#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/byte_multi_array.hpp>

#include <linux/spi/spidev.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <unistd.h>
    
#include <thread>
#include <string>
#include <vector>
#include <cstring>
#include <cerrno>

class SpiNode : public rclcpp::Node
{
public:
    SpiNode() : Node("spi_node")
    {
        this->declare_parameter<std::string>("device", "/dev/spidev0.0");
        this->declare_parameter<int>("speed_hz", 660000);
        this->get_parameter("device", device_);
        this->get_parameter("speed_hz", speed_hz_);

        publisher_ = this->create_publisher<std_msgs::msg::ByteMultiArray>("spi_rx", 10);
        subscriber_ = this->create_subscription<std_msgs::msg::ByteMultiArray>(
            "spi_tx", 10, std::bind(&SpiNode::handle_outgoing_message, this, std::placeholders::_1));

        if (connect_to_spi())
        {
            read_thread_ = std::thread([this]() { read_from_spi(); });
        }
    }

    ~SpiNode()
    {
        running_ = false;
        if (read_thread_.joinable())
            read_thread_.join();
        if (fd_ >= 0)
            close(fd_);
    }

private:
    int fd_ = -1;
    std::string device_;
    int speed_hz_;
    bool running_ = true;
    std::thread read_thread_;

    rclcpp::Publisher<std_msgs::msg::ByteMultiArray>::SharedPtr publisher_;
    rclcpp::Subscription<std_msgs::msg::ByteMultiArray>::SharedPtr subscriber_;

    bool connect_to_spi()
    {
        fd_ = open(device_.c_str(), O_RDWR);
        if (fd_ < 0)
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to open SPI device %s: %s", device_.c_str(), std::strerror(errno));


            return false;
        }

        uint8_t mode = SPI_MODE_0;
        uint8_t bits = 8;
        if (ioctl(fd_, SPI_IOC_WR_MODE, &mode) < 0 ||
            ioctl(fd_, SPI_IOC_WR_BITS_PER_WORD, &bits) < 0 ||
            ioctl(fd_, SPI_IOC_WR_MAX_SPEED_HZ, &speed_hz_) < 0)
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to configure SPI");
            close(fd_);
            fd_ = -1;
            return false;
        }

        RCLCPP_INFO(this->get_logger(), "SPI connected: %s, speed %d Hz", device_.c_str(), speed_hz_);
        return true;
    }

    void read_from_spi()
    {
        while (running_ && rclcpp::ok())
        {
            uint8_t tx[1] = {0x00};   // dummy byte to generate clock
            uint8_t rx[1] = {0x00};

            spi_ioc_transfer tr{};
            tr.tx_buf = (unsigned long)tx;
            tr.rx_buf = (unsigned long)rx;
            tr.len = 1;
            tr.speed_hz = speed_hz_;
            tr.bits_per_word = 8;
            // tr.cs_change = 0; // keep default

            int ret = ioctl(fd_, SPI_IOC_MESSAGE(1), &tr);
            if (ret < 0)
            {
                RCLCPP_ERROR(this->get_logger(), "SPI read transfer failed: %s", std::strerror(errno));
            }
            else
            {
                uint8_t b = rx[0];
                std_msgs::msg::ByteMultiArray msg;
                msg.data.push_back(b);
                publisher_->publish(msg);

                RCLCPP_INFO(this->get_logger(),
                    "RX: 0x%02X  DEC:%u  BIN:%u%u%u%u%u%u%u%u",
                    b, b,
                    (b >> 7) & 1, (b >> 6) & 1, (b >> 5) & 1, (b >> 4) & 1,
                    (b >> 3) & 1, (b >> 2) & 1, (b >> 1) & 1, (b >> 0) & 1
                );
            }

            // std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
    }

    void handle_outgoing_message(const std_msgs::msg::ByteMultiArray::SharedPtr msg)
    {
        if (fd_ < 0)
        {
            RCLCPP_WARN(this->get_logger(), "SPI device not open");
            return;
        }

        struct spi_ioc_transfer tr = {};
        tr.tx_buf = (unsigned long)msg->data.data();
        tr.len = msg->data.size();
        tr.speed_hz = speed_hz_;
        tr.bits_per_word = 8;

        if (ioctl(fd_, SPI_IOC_MESSAGE(1), &tr) < 0)
        {
            RCLCPP_ERROR(this->get_logger(), "SPI write failed");
        }
        else
        {
            RCLCPP_INFO(this->get_logger(), "Sent %zu bytes via SPI", msg->data.size());
        }
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<SpiNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
