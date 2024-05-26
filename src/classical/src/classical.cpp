#include <memory>

#include <algorithm>
#include <cmath>
#include <tuple>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/u_int16.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "realsense2_camera_msgs/msg/rgbd.hpp"
#include "cv_bridge/cv_bridge.h"
#include "opencv2/opencv.hpp"
#include "opencv2/highgui/highgui.hpp"

using std::placeholders::_1;
using namespace cv;
using namespace std;

struct ArmorPlate {
    Point tl;
    Point br;
    Point center;
};

class Classical : public rclcpp::Node
{
  public:
    Classical()
    : Node("classical")
    {
        this->declare_parameter("target_color", "red");
        this->declare_parameter("enable_debug", true);
        subscription_ = this->create_subscription<realsense2_camera_msgs::msg::RGBD>(
            "/robot/rs2/rgbd", 10, std::bind(&Classical::topic_callback, this, _1));
        image_result_ = this->create_publisher<sensor_msgs::msg::Image>("image_result", 10);
        depth_result_ = this->create_publisher<std_msgs::msg::UInt16>("depth_result", 10);


    }

  private:
    static bool rect_sort_function (Rect first, Rect second)
    {
        return first.tl().x < second.tl().x;
    }

    Mat mask_color(Mat hsv_img) {
        Mat color_mask;
        if (this->get_parameter("target_color").as_string() == "red") {
            Mat mask1, mask2;
            inRange(hsv_img, Scalar(redLowH_1, redLowS_1, redLowV_1), Scalar(redHighH_1, redHighS_1, redHighV_1), mask1);
            inRange(hsv_img, Scalar(redLowH_2, redLowS_2, redLowV_2), Scalar(redHighH_2, redHighS_2, redHighV_2), mask2);
            color_mask = mask1 | mask2;
        }

        else {
            inRange(hsv_img, Scalar(blueLowH, blueLowS, blueLowV), Scalar(blueHighH, blueHighS, blueHighV), color_mask); //Threshold the image
        }

        return color_mask;
    }

    Mat remove_artifacts(Mat img) {
        morphologyEx(img, img, MORPH_OPEN, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)));
        morphologyEx(img, img, MORPH_CLOSE, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)));
        return img;
    }

    std::tuple<vector<Rect>, vector<vector<Point>>> find_bounding_boxes(Mat img) {
        vector<vector<Point>> contours;
        vector<Vec4i> hierarchy;
        findContours(img, contours, hierarchy, RETR_EXTERNAL, CHAIN_APPROX_NONE);

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
        return std::make_tuple(accepted_rects, contours);
    }

    std::tuple<vector<ArmorPlate>, vector<Rect>> find_armor_plates(vector<Rect> bounding_boxes) {
        vector<ArmorPlate> armor_plates;
        if (bounding_boxes.size() >= 2)
        {
            Rect first = bounding_boxes[0];
            Rect second = bounding_boxes[1];

            int distance = abs(second.tl().y - first.tl().y);

            for (size_t i = 1; i + 1 < bounding_boxes.size(); i++)
            {
                // if (bounding_boxes[i].tl().x - first_tallest.tl().x < 1000)
                    // continue;

                if (abs(bounding_boxes[i].tl().y - bounding_boxes[i + 1].tl().y) < distance)
                {
                    distance = abs(bounding_boxes[i].tl().y - bounding_boxes[i + 1].tl().y);
                    first = bounding_boxes[i];
                    second = bounding_boxes[i + 1];
                }

                // rectangle(image_copy, first.tl(), second.br(), Scalar(0, 255, 0), 5);   
                int x = (first.tl().x + second.br().x) / 2;
                int y = (first.tl().y + second.br().y) / 2;
                // circle(image_copy, Point(x, y), 2, Scalar(0, 0, 255), 8);
                armor_plates.push_back(ArmorPlate{first.tl(), second.br(), Point(x,y)});
            }
        }

        return std::make_tuple(armor_plates, bounding_boxes);
    }

    void topic_callback(const realsense2_camera_msgs::msg::RGBD & img)
    {
        cv_bridge::CvImagePtr cv_rgb_ptr = cv_bridge::toCvCopy(img.rgb, sensor_msgs::image_encodings::BGR8);
        cv_bridge::CvImagePtr cv_depth_ptr = cv_bridge::toCvCopy(img.depth);

        Mat rgb_img = cv_rgb_ptr->image;
        Mat depth_img = cv_depth_ptr->image;

        Mat hsv_img;
        cvtColor(rgb_img, hsv_img, COLOR_BGR2HSV); //Convert the captured frame from BGR to HSV
        
        Mat color_mask = mask_color(hsv_img);
        color_mask = remove_artifacts(color_mask);

        Mat image_copy = rgb_img.clone();
        Mat image_contours = rgb_img.clone();
        Mat image_all_bounded_boxes = rgb_img.clone();

        auto [accepted_rects, contours] = find_bounding_boxes(color_mask);
        
        auto [armor_plates, bounding_boxes] = find_armor_plates(accepted_rects);

        if (this->get_parameter("enable_debug").as_bool() == true) {
            for (size_t i = 0; i < bounding_boxes.size(); i++) // iterate through each contour.
            {
                rectangle(image_all_bounded_boxes, bounding_boxes[i].tl(), bounding_boxes[i].br(), Scalar(0, 255, 0), 5); 
            }

            for (size_t i = 0; i < armor_plates.size(); i++) {
                rectangle(image_all_bounded_boxes, armor_plates[i].tl, armor_plates[i].br, Scalar(255, 0, 0), 5);
                circle(image_all_bounded_boxes, armor_plates[i].center, 2, Scalar(0, 0, 255), 8);
            }

            sensor_msgs::msg::Image msg = *cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", image_all_bounded_boxes).toImageMsg();
            std_msgs::msg::UInt16 depth_msg; 
            depth_msg.data = depth_img.at<uint16_t>(armor_plates[0].center);

            image_result_->publish(msg);
            depth_result_->publish(depth_msg);
        }
    }
    rclcpp::Subscription<realsense2_camera_msgs::msg::RGBD>::SharedPtr subscription_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_result_;
    rclcpp::Publisher<std_msgs::msg::UInt16>::SharedPtr depth_result_;

    /** Constants **/

    // Blue mask
    uint8_t blueLowH = 80;
    uint8_t blueHighH = 140;
    uint8_t blueLowS = 100; 
    uint8_t blueHighS = 255;
    uint8_t blueLowV = 175;
    uint8_t blueHighV = 255;

    // Red mask
    uint8_t redLowH_1 = 0;
    uint8_t redHighH_1 = 10;
    uint8_t redLowS_1 = 70; 
    uint8_t redHighS_1 = 255;
    uint8_t redLowV_1 = 50;
    uint8_t redHighV_1 = 255;

    uint8_t redLowH_2 = 170;
    uint8_t redHighH_2 = 180;
    uint8_t redLowS_2 = 70; 
    uint8_t redHighS_2 = 255;
    uint8_t redLowV_2 = 50;
    uint8_t redHighV_2 = 255;
};



int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Classical>());
  rclcpp::shutdown();
  return 0;
}