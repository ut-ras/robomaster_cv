#include <memory>

#include <algorithm>
#include <cmath>
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "cv_bridge/cv_bridge.h"
#include "opencv2/opencv.hpp"
#include "opencv2/highgui/highgui.hpp"

using std::placeholders::_1;
using namespace cv;
using namespace std;

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
    static bool rect_sort_function (Rect first, Rect second)
    {
        return first.tl().x < second.tl().x;
    }

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

        Mat image_copy = imgOrig.clone();
        Mat image_contours = imgOrig.clone();
        Mat image_all_bounded_boxes = imgOrig.clone();

        vector<vector<Point>> contours;
        vector<Vec4i> hierarchy;
        findContours(color_threshold, contours, hierarchy, RETR_EXTERNAL, CHAIN_APPROX_NONE);
        drawContours(image_contours, contours, -1, Scalar(0, 255, 0), 2);

        vector<Rect> accepted_rects;
        for (size_t i = 0; i < contours.size(); i++) // iterate through each contour.
        {
            Rect bounding_rect = boundingRect(contours[i]);
            if (bounding_rect.size().height / bounding_rect.size().width > 1
                && bounding_rect.size().height / bounding_rect.size().width < 7)
            {
                accepted_rects.push_back(bounding_rect);
            }
        }

        sort(accepted_rects.begin(), accepted_rects.end(), rect_sort_function);

        if (accepted_rects.size() >= 2)
        {
            Rect first = accepted_rects[0];
            Rect second = accepted_rects[1];

            int distance = abs(second.tl().y - first.tl().y);

            for (size_t i = 1; i + 1 < accepted_rects.size(); i++)
            {
                // if (accepted_rects[i].tl().x - first_tallest.tl().x < 1000)
                    // continue;

                if (abs(accepted_rects[i].tl().y - accepted_rects[i + 1].tl().y) < distance)
                {
                    distance = abs(accepted_rects[i].tl().y - accepted_rects[i + 1].tl().y);
                    first = accepted_rects[i];
                    second = accepted_rects[i + 1];
                }
            }

            rectangle(image_copy, first.tl(), second.br(), Scalar(0, 255, 0), 5);   
            // rectangle(image_copy, second.tl(), second.br(), Scalar(0, 255, 0), 5);  
            float x = (first.tl().x + second.br().x) / 2;
            float y = (first.tl().y + second.br().y) / 2;
            circle(image_copy, Point(x, y), 2, Scalar(0, 0, 255), 8);

            for (size_t i = 0; i < accepted_rects.size(); i++) // iterate through each contour.
            {
                rectangle(image_all_bounded_boxes, accepted_rects[i].tl(), accepted_rects[i].br(), Scalar(0, 255, 0), 5); 
            }

            for (size_t i = 0; i < accepted_rects.size() - 1; i++)
            {
                rectangle(image_all_bounded_boxes, accepted_rects[i].tl(), accepted_rects[i+1].br(), Scalar(255, 0, 0), 5);
                float center_x = (accepted_rects[i].tl().x + accepted_rects[i+1].br().x) / 2;
                float center_y = (accepted_rects[i].tl().y + accepted_rects[i+1].br().y) / 2;
                circle(image_all_bounded_boxes, Point(center_x, center_y), 2, Scalar(255, 0, 0), 5);
            }
        }

        sensor_msgs::msg::Image msg = *cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", image_all_bounded_boxes).toImageMsg();

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