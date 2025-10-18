#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/core.hpp>
#include <opencv2/videoio.hpp>

using namespace cv;
using namespace std::chrono_literals;

class VideoPublisher : public rclcpp::Node {
public:
    VideoPublisher() : Node("video_publisher"), cap("test.mp4") {
        if (!cap.isOpened()) {
            RCLCPP_ERROR(this->get_logger(), "Failed to open video file!");
            rclcpp::shutdown();
        }

        publisher_ = this->create_publisher<sensor_msgs::msg::CompressedImage>("image_raw/compressed", 10);
        timer_ = this->create_wall_timer(33ms, std::bind(&VideoPublisher::publish_frame, this));
    }

private:
    VideoCapture cap;
    rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;

    void publish_frame() {
        Mat frame;
        if (!cap.read(frame)) {
            RCLCPP_INFO(this->get_logger(), "Video playback complete. Stopping publisher.");
            rclcpp::shutdown();
            return;
        }

        std_msgs::msg::Header header;
        header.stamp = this->now();
        auto msg = cv_bridge::CvImage(header, "bgr8", frame).toCompressedImageMsg(cv_bridge::Format::JPEG);
        publisher_->publish(*msg);
    }
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<VideoPublisher>());
    rclcpp::shutdown();
    return 0;
}