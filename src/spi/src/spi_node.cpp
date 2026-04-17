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

using namespace std::chrono_literals;



class FtdiSpiNode : public rclcpp::Node
{
public:
    FtdiSpiNode() : Node("ftdi_spi_node")
    {
        this->declare_parameter<int>("vendor_id", 0x0403);    // this is the first part of the usb id run lsusb to get the list of active usb ids
        this->declare_parameter<int>("product_id", 0x6014);   // FT232H default (second part of the usb id)
        this->declare_parameter<int>("clock_divisor", 29);    // ~1 MHz for 60 MHz base
        this->declare_parameter<int>("poll_ms", 10);
        this->declare_parameter<int>("transfer_len", 1);

        


        vendor_id_ = this->get_parameter("vendor_id").as_int();
        product_id_ = this->get_parameter("product_id").as_int();
        clock_divisor_ = this->get_parameter("clock_divisor").as_int();
        poll_ms_ = this->get_parameter("poll_ms").as_int();
        transfer_len_ = this->get_parameter("transfer_len").as_int();

        publisher_ = this->create_publisher<std_msgs::msg::ByteMultiArray>("spi_rx", 10);

        open_and_configure();

        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(poll_ms_),
            std::bind(&FtdiSpiNode::poll_once, this));

        RCLCPP_INFO(this->get_logger(),
                    "FTDI SPI node started. VID=0x%04X PID=0x%04X len=%d poll=%d ms",
                    vendor_id_, product_id_, transfer_len_, poll_ms_);
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

    void poll_once()
    {
        try {
            std::vector<uint8_t> tx(transfer_len_, counter_++); // this is the data that we are sending back
            auto rx = spi_exchange(tx);

            std_msgs::msg::ByteMultiArray msg;
            msg.data.assign(rx.begin(), rx.end());
            publisher_->publish(msg);

            std::ostringstream hex_line;
            hex_line << "RAW HEX:";
            for (auto b : rx) {
                hex_line << " "
                         << std::uppercase << std::hex << std::setw(2)
                         << std::setfill('0') << static_cast<int>(b);
            }
            // RCLCPP_INFO(this->get_logger(), "%s", hex_line.str().c_str());

            for (size_t i = 0; i < rx.size(); ++i) {
                if (static_cast<int>(rx[i]) == 70){
                    RCLCPP_INFO(this->get_logger(), "BEGINNING MESSAGE STREAM");
                }

                RCLCPP_INFO(this->get_logger(),
                            "byte[%zu] = 0x%02X = %s = %d",
                            i, rx[i], bits(rx[i]).c_str(), static_cast<int>(rx[i]));
                
                if (static_cast<int>(rx[i]) == 57){
                    RCLCPP_INFO(this->get_logger(), "END OF MESSAGE STREAM");
                }
            }

            std::ostringstream bitstream;
            bitstream << "BITSTREAM:";
            for (auto b : rx) {
                bitstream << " " << bits(b);
            }
            // RCLCPP_INFO(this->get_logger(), "%s", bitstream.str().c_str());
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "poll_once failed: %s", e.what());
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
    int poll_ms_ = 10;
    int transfer_len_ = 1;
    uint8_t counter_ = 0;

    rclcpp::Publisher<std_msgs::msg::ByteMultiArray>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
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