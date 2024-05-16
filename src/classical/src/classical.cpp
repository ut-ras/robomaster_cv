#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "cv_bridge/cv_bridge.h"
#include "opencv2/opencv.hpp"
#include "opencv2/highgui/highgui.hpp"

using std::placeholders::_1;
using namespace cv;

class Classical : public rclcpp::Node
{
  public:
    Classical()
    : Node("classical")
    {
        subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
            "/robot/rs2/color/image_raw", 10, std::bind(&Classical::topic_callback, this, _1));
        publisher_ = this->create_publisher<sensor_msgs::msg::Image>("image_result", 10);
    }

  private:
    void topic_callback(const sensor_msgs::msg::Image & img)
    {
        // Blue current using low 80 high 140
        int iLowH = 80;
        int iHighH = 140;

        // Blue current using low 100 high 255
        int iLowS = 100; 
        int iHighS = 255;

        // Blue current using low 175 high 255
        int iLowV = 175;
        int iHighV = 255;

        cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(img, sensor_msgs::image_encodings::BGR8);

        Mat imgOrig = cv_ptr->image;

        Mat imgHSV;
        cvtColor(imgOrig, imgHSV, COLOR_BGR2HSV); //Convert the captured frame from BGR to HSV
        Mat color_threshold;
        inRange(imgHSV, Scalar(iLowH, iLowS, iLowV), Scalar(iHighH, iHighS, iHighV), color_threshold); //Threshold the image

        // Red Mask
        Mat mask1, mask2;
        inRange(imgHSV, Scalar(0, 70, 50), Scalar(10, 255, 255), mask1);
        inRange(imgHSV, Scalar(170, 70, 50), Scalar(180, 255, 255), mask2);
        color_threshold = mask1 | mask2;
        
        //morphological opening (removes small objects from the foreground)
        erode(color_threshold, color_threshold, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)) );
        dilate(color_threshold, color_threshold, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)) ); 

        //morphological closing (removes small holes from the foreground)
        dilate(color_threshold, color_threshold, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)) ); 
        erode(color_threshold, color_threshold, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)) );

        Mat image_result;
        cvtColor(color_threshold, image_result, COLOR_GRAY2BGR);

        sensor_msgs::msg::Image msg = *cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", image_result).toImageMsg();

        publisher_->publish(msg);
    }
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Classical>());
  rclcpp::shutdown();
  return 0;
}