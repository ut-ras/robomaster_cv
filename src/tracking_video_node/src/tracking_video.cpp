#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/core.hpp>
#include <vision_msgs/msg/detection2_d_array.hpp>
#include <vision_msgs/msg/detection2_d.hpp>
#include <vision_msgs/msg/object_hypothesis_with_pose.hpp>
#include <opencv2/videoio.hpp>
#include <rclcpp/parameter.hpp>

using std::placeholders::_1;
using std::placeholders::_2;

using vision_msgs::msg::Detection2DArray;
using sensor_msgs::msg::CompressedImage;
using namespace cv;
using namespace std;
using vision_msgs::msg::Detection2DArray;
using vision_msgs::msg::Detection2D;
using vision_msgs::msg::ObjectHypothesisWithPose;

Mat applyCanny(Mat& frame) {
    Mat edges;
    GaussianBlur(frame, frame, Size(5, 5), 1.5);
    // Canny with thresholds 100 and 200
    Canny(frame, edges, 100, 200);
    Mat edgesColor;
    cvtColor(edges, edgesColor, COLOR_GRAY2BGR);
    return edgesColor;
}

class DetectionListener : public rclcpp::Node
{
public:
    DetectionListener() : Node("detection_listener")
    {
        detection_sub_.subscribe(this, "predicted_points");
        image_sub_.subscribe(this, "/robot/rs2/color/image_raw/compressed");

        sync_ = std::make_shared<Sync>(SyncPolicy(10), detection_sub_, image_sub_);
        sync_->registerCallback(std::bind(&DetectionListener::callback, this, _1, _2));
    }

private:
    bool writer_initialized = false; 
    cv::VideoWriter writer_; 

    int frame_number = 0; 

    void callback(const Detection2DArray::ConstSharedPtr detection_msg,
                  const CompressedImage::ConstSharedPtr image_msg)
    {
            frame_number++;


            // RCLCPP_INFO(this->get_logger(), "Detection - x: %.2f, y: %.2f, Predicted x: %.2f, predicted y: %.2f",
            //             detectedX, detectedY, predictedX, predictedY);

            // Optional: use `image_msg` here
            try {

              cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(image_msg, sensor_msgs::image_encodings::BGR8);
              Mat frame = cv_ptr->image;

              Mat edges = applyCanny(frame);
              
              if (!writer_initialized) {
                  writer_.open("predicted_video.mp4", cv::VideoWriter::fourcc('m','p','4','v'), 30, frame.size(), true);
                  if (!writer_.isOpened()) {
                      RCLCPP_ERROR(this->get_logger(), "Could not open the output video for write");
                  }
                  writer_initialized = true;
              }

            float predictedX; 
            float predictedY;
            float detectedX;
            float detectedY;
            if (detection_msg->detections.size() >= 2)
            {
                predictedX = detection_msg->detections[0].bbox.center.position.x;
                predictedY = detection_msg->detections[0].bbox.center.position.y;

                detectedX = detection_msg->detections[1].bbox.center.position.x;
                detectedY = detection_msg->detections[1].bbox.center.position.y;

                 // draw two circles right here :) O O (like that <- :) )
                auto actualPoint = cv::Point();
                actualPoint.x = detectedX; 
                actualPoint.y = detectedY; 

                circle(frame, actualPoint, 10, Scalar(255, 0, 255), -1);

                auto predictedPoint = cv::Point();
                predictedPoint.x = predictedX; 
                predictedPoint.y = predictedY; 

                circle(frame, predictedPoint, 10, Scalar(0, 255, 0), -1);
                RCLCPP_INFO(this->get_logger(), "Framer Number: %d, Detection - x: %.2f, y: %.2f, Predicted x: %.2f, predicted y: %.2f", frame_number, detectedX, detectedY, predictedX, predictedY);
              
                

            }
            else if (detection_msg->detections.size() == 1){
                predictedX = detection_msg->detections[0].bbox.center.position.x;
                predictedY = detection_msg->detections[0].bbox.center.position.y;

                auto predictedPoint = cv::Point();
                predictedPoint.x = predictedX; 
                predictedPoint.y = predictedY; 

                circle(frame, predictedPoint, 10, Scalar(0, 255, 0), -1);
                RCLCPP_INFO(this->get_logger(), "Framer Number: %d, Predicted x: %.2f, predicted y: %.2f", frame_number, predictedX, predictedY);
            } else {
                RCLCPP_INFO(this->get_logger(), "Frame Number: %d, No detections", frame_number);
            }


            writer_.write(frame);
            // RCLCPP_INFO(this->get_logger(), "Frame Number: %d", frame_number);

            } catch (cv_bridge::Exception &e) {
                RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
            }
        
    }

    message_filters::Subscriber<Detection2DArray> detection_sub_;
    message_filters::Subscriber<CompressedImage> image_sub_;

    using SyncPolicy = message_filters::sync_policies::ApproximateTime<Detection2DArray, CompressedImage>;
    using Sync = message_filters::Synchronizer<SyncPolicy>;
    std::shared_ptr<Sync> sync_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<DetectionListener>());
    rclcpp::shutdown();
    return 0;
}