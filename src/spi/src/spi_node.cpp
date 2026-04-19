#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/byte_multi_array.hpp>

#include <libftdi1/ftdi.h>

#include <chrono>
#include <cstdint>
#include <cstdlib>
#include <iomanip>
#include <iostream>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>
#include <bit>

using namespace std::chrono_literals;



class FtdiSpiNode : public rclcpp::Node
{
public:
    FtdiSpiNode() : Node("ftdi_spi_node")
    {
        this->declare_parameter<int>("vendor_id", 0x0403);    // this is the first part of the usb id run lsusb to get the list of active usb ids
        this->declare_parameter<int>("product_id", 0x6014);   // FT232H default (second part of the usb id)
        this->declare_parameter<int>("clock_divisor", 29);    // ~1 MHz for 60 MHz base
        this->declare_parameter<int>("transfer_len", 1);
        this->declare_parameter<int>("start_byte", 70);
        this->declare_parameter<int>("end_byte", 57);
        this->declare_parameter<int>("max_message_len", 13);

        vendor_id_ = this->get_parameter("vendor_id").as_int();
        product_id_ = this->get_parameter("product_id").as_int();
        clock_divisor_ = this->get_parameter("clock_divisor").as_int();
        transfer_len_ = this->get_parameter("transfer_len").as_int();
        start_byte_ = this->get_parameter("start_byte").as_int();
        end_byte_ = this->get_parameter("end_byte").as_int();
        max_message_len_ = this->get_parameter("max_message_len").as_int();

        publisher_ = this->create_publisher<std_msgs::msg::ByteMultiArray>("spi_rx", 10);

        open_and_configure();

        packing_sub_ = this->create_subscription<std_msgs::msg::ByteMultiArray>(
            "spi_tx", 10,
            [this](const std_msgs::msg::ByteMultiArray::SharedPtr msg) {
                spi_tx_callback(msg);
            });

        RCLCPP_INFO(this->get_logger(),
                    "FTDI SPI node started. VID=0x%04X PID=0x%04X transfer_len=%d",
                    vendor_id_, product_id_, transfer_len_);
    }

    ~FtdiSpiNode() override
    {
        cleanup();
    }

private:
    static constexpr uint8_t PIN_SK = 1 << 0;   // AD0
    static constexpr uint8_t PIN_DO = 1 << 1;   // AD1 MOSI
    static constexpr uint8_t PIN_DI = 1 << 2;   // AD2 MISO
    static constexpr uint8_t PIN_CS = 1 << 3;   // AD3 CS

    static constexpr uint8_t DIR = PIN_SK | PIN_DO | PIN_CS;
    static constexpr uint8_t CS_IDLE = PIN_CS;   // high
    static constexpr uint8_t CS_ACTIVE = 0x00;   // low

    void check_ok(int rc, const char* what)
    {
        if (rc < 0) {
            std::ostringstream oss;
            oss << what << " failed: " << ftdi_get_error_string(ftdi_);
            throw std::runtime_error(oss.str());
        }
    }

    void write_all(const std::vector<uint8_t>& data)
    {
        int offset = 0;
        while (offset < static_cast<int>(data.size())) {
            int n = ftdi_write_data(
                ftdi_,
                const_cast<unsigned char*>(data.data()) + offset,
                static_cast<int>(data.size()) - offset);
            check_ok(n, "ftdi_write_data");
            offset += n;
        }
    }

    std::vector<uint8_t> read_exact(int nbytes)
    {
        std::vector<uint8_t> out(nbytes);
        int offset = 0;

        while (offset < nbytes) {
            int n = ftdi_read_data(ftdi_, out.data() + offset, nbytes - offset);
            check_ok(n, "ftdi_read_data");

            if (n == 0) {
                rclcpp::sleep_for(1ms);
                continue;
            }
            offset += n;
        }
        return out;
    }

    std::string bits(uint8_t b) const
    {
        std::string s;
        for (int i = 7; i >= 0; --i) {
            s.push_back((b & (1u << i)) ? '1' : '0');
        }
        return s;
    }

    void open_and_configure()
    {
        ftdi_ = ftdi_new();
        if (!ftdi_) {
            throw std::runtime_error("ftdi_new failed");
        }

        check_ok(ftdi_usb_open(ftdi_, vendor_id_, product_id_), "ftdi_usb_open");
        check_ok(ftdi_usb_reset(ftdi_), "ftdi_usb_reset");
        check_ok(ftdi_set_interface(ftdi_, INTERFACE_ANY), "ftdi_set_interface");
        check_ok(ftdi_tcioflush(ftdi_), "ftdi_tcioflush");
        check_ok(ftdi_set_latency_timer(ftdi_, 1), "ftdi_set_latency_timer");

        check_ok(ftdi_set_bitmode(ftdi_, 0x00, BITMODE_RESET), "ftdi_set_bitmode reset");
        check_ok(ftdi_set_bitmode(ftdi_, 0x00, BITMODE_MPSSE), "ftdi_set_bitmode mpsse");

        rclcpp::sleep_for(50ms);

        // MPSSE sync
        write_all({0xAA});
        auto sync = read_exact(2);
        if (sync[0] != 0xFA || sync[1] != 0xAA) {
            std::ostringstream oss;
            oss << "MPSSE sync failed: got 0x"
                << std::hex << std::setw(2) << std::setfill('0') << int(sync[0])
                << " 0x" << std::setw(2) << int(sync[1]);
            throw std::runtime_error(oss.str());
        }

        // H-series setup
        write_all({0x8A});                         // disable divide-by-5
        write_all({0x97, 0x8D});                   // disable adaptive and 3-phase clock
        write_all({0x86,
                   static_cast<uint8_t>(clock_divisor_ & 0xFF),
                   static_cast<uint8_t>((clock_divisor_ >> 8) & 0xFF)});  // clock divisor
        write_all({0x80, CS_IDLE, DIR});          // idle pins
    }

    std::vector<uint8_t> spi_exchange(const std::vector<uint8_t>& tx)
    {
        std::vector<uint8_t> cmd;

        cmd.push_back(0x80);       // set low-byte pins
        cmd.push_back(CS_ACTIVE);  // CS low
        cmd.push_back(DIR);

        cmd.push_back(0x31);       // clock bytes out+in, MSB first
        cmd.push_back(static_cast<uint8_t>((tx.size() - 1) & 0xFF));
        cmd.push_back(static_cast<uint8_t>(((tx.size() - 1) >> 8) & 0xFF));
        cmd.insert(cmd.end(), tx.begin(), tx.end());

        cmd.push_back(0x80);       // set low-byte pins
        cmd.push_back(CS_IDLE);    // CS high
        cmd.push_back(DIR);

        cmd.push_back(0x87);       // send immediate

        write_all(cmd);
        // RCLCPP_INFO(this->get_logger(), "Sent SPI command with %zu bytes", tx.size());
        // RCLCPP_INFO(this->get_logger(), "Sending: %s : %s: %s: %s: %s: %s: %s: %s", bits(tx[0]).c_str(), bits(tx[1]).c_str(), bits(tx[2]).c_str(), bits(tx[3]).c_str(), bits(tx[4]).c_str(), bits(tx[5]).c_str(), bits(tx[6]).c_str(), bits(tx[7]).c_str());
        // RCLCPP_INFO(this->get_logger(), "Sent Bytes: %s", bits(tx[0]).c_str());
        return read_exact(static_cast<int>(tx.size()));
    }

    void spi_tx_callback(const std_msgs::msg::ByteMultiArray::SharedPtr msg)
    {
        if (msg->data.empty()) {
            RCLCPP_WARN(this->get_logger(), "spi_tx received empty message, ignoring");
            return;
        }

        try {
            auto rx = spi_exchange(msg->data);

            std::ostringstream hex_line;
            hex_line << "RAW HEX:";
            for (auto b : rx) {
                hex_line << " "
                         << std::uppercase << std::hex << std::setw(2)
                         << std::setfill('0') << static_cast<int>(b);
            }

            for (const auto& byte : rx) {
                if (!in_message_ && static_cast<int>(byte) == start_byte_) {
                    RCLCPP_INFO(this->get_logger(), "BEGINNING MESSAGE STREAM");
                    in_message_ = true;
                } else if (in_message_ && static_cast<int>(byte) == end_byte_) {
                    RCLCPP_INFO(this->get_logger(), "END OF MESSAGE STREAM");
                    in_message_ = false;
                    std_msgs::msg::ByteMultiArray out_msg;
                    out_msg.data.assign(received_data_.begin(), received_data_.end());
                    publisher_->publish(out_msg);
                    received_data_.clear();
                } else if (in_message_ && received_data_.size() >= static_cast<size_t>(max_message_len_)) {
                    in_message_ = false;
                    RCLCPP_WARN(this->get_logger(), "Message too long, resetting. Received data: %s",
                                hex_line.str().c_str());
                    received_data_.clear();
                } else if (in_message_) {
                    RCLCPP_INFO(this->get_logger(), "byte[%zu] = 0x%02X = %s = %d",
                                received_data_.size(), byte, bits(byte).c_str(), static_cast<int>(byte));
                    received_data_.push_back(byte);
                }
            }
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "spi_tx_callback failed: %s", e.what());
        }
    }

    void cleanup()
    {
        if (ftdi_) {
            ftdi_usb_close(ftdi_);
            ftdi_free(ftdi_);
            ftdi_ = nullptr;
        }
    }

    struct ftdi_context* ftdi_ = nullptr;

    int vendor_id_ = 0;
    int product_id_ = 0;
    int clock_divisor_ = 29;
    int transfer_len_ = 1;
    int start_byte_ = 70;
    int end_byte_ = 57;
    int max_message_len_ = 13;
    bool in_message_ = false;
    std::vector<uint8_t> received_data_;

    rclcpp::Publisher<std_msgs::msg::ByteMultiArray>::SharedPtr publisher_;
    rclcpp::Subscription<std_msgs::msg::ByteMultiArray>::SharedPtr packing_sub_;
};

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    try {
        auto node = std::make_shared<FtdiSpiNode>();
        rclcpp::spin(node);
    } catch (const std::exception& e) {
        std::cerr << "Fatal error: " << e.what() << std::endl;
    }
    rclcpp::shutdown();
    return 0;
}