#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

using namespace cv;
using namespace std::chrono_literals;

class VideoPublisher : public rclcpp::Node {
public:
    VideoPublisher() : Node("video_publisher"), cap("test.mp4") {
        if (!cap.isOpened()) {
            RCLCPP_ERROR(this->get_logger(), "Failed to open video file!");
            rclcpp::shutdown();
        }

        publisher_ = this->create_publisher<sensor_msgs::msg::Image>("image_raw", 10);
        timer_ = this->create_wall_timer(33ms, std::bind(&VideoPublisher::publish_frame, this));
    }

private:
    VideoCapture cap;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;

    void publish_frame() {
        Mat frame;
        if (!cap.read(frame)) {
            RCLCPP_INFO(this->get_logger(), "Video playback complete. Stopping publisher.");
            rclcpp::shutdown();
            return;
        }

        auto msg = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", frame).toImageMsg();
        publisher_->publish(*msg);
    }
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<VideoPublisher>());
    rclcpp::shutdown();
    return 0;
}
