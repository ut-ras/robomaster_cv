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
#include "stampede_msgs/msg/object_log_input.hpp"
#include "stampede_msgs/msg/bounding_box.hpp"

using std::placeholders::_1;
using namespace cv;
using namespace std;

struct ArmorPlate {
    Point tl;
    Point br;
    Point center;
    RotatedRect left_light;
    RotatedRect right_light;
};

struct left_right_contour_sorter // 'less' for contours
{
    bool operator ()( const vector<Point>& a, const vector<Point> & b )
    {
        Rect ra(boundingRect(a));
        Rect rb(boundingRect(b));
        return (ra.x < rb.x);
    }
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
        classical_output_ = this->create_publisher<stampede_msgs::msg::ObjectLogInput>("object_log_input", 10);
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
        // morphologyEx(img, img, MORPH_OPEN, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)));
        // morphologyEx(img, img, MORPH_CLOSE, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)));

        //morphological opening (removes small objects from the foreground)
        erode(img, img, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)) );
        dilate(img, img, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)) ); 

        //morphological closing (removes small holes from the foreground)
        dilate(img, img, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)) ); 
        erode(img, img, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)) );
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

    std::tuple<vector<RotatedRect>, vector<vector<Point>>> find_rotated_bounding_boxes(Mat img) {
        vector<vector<Point>> contours;
        findContours(img, contours, RETR_EXTERNAL, CHAIN_APPROX_NONE);
        sort(contours.begin(), contours.end(), left_right_contour_sorter());

        vector<RotatedRect> bounding_boxes;
        for (size_t i = 0; i < contours.size(); i++) {
            RotatedRect bounding_box = minAreaRect(contours[i]);

            // if (bounding_box.boundingRect().height / bounding_box.boundingRect().width > 1 && bounding_box.boundingRect().height / bounding_box.boundingRect().width < 7) {
                bounding_boxes.push_back(bounding_box);
            // }
        }

        return std::make_tuple(bounding_boxes, contours);
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
            }

            // rectangle(image_copy, first.tl(), second.br(), Scalar(0, 255, 0), 5);   
            int x = (first.tl().x + second.br().x) / 2;
            int y = (first.tl().y + second.br().y) / 2;
            // circle(image_copy, Point(x, y), 2, Scalar(0, 0, 255), 8);
            armor_plates.push_back(ArmorPlate{first.tl(), second.br(), Point(x,y)});
        }

        return std::make_tuple(armor_plates, bounding_boxes);
    }

    float normalize_0_180(RotatedRect bounding_box) {
        if (bounding_box.size.width < bounding_box.size.height) {
            return (90 - bounding_box.angle);
        }

        return -bounding_box.angle;
    }

    std::tuple<vector<ArmorPlate>, vector<RotatedRect>> find_rotated_armor_plates(vector<RotatedRect> bounding_boxes) {
        vector<ArmorPlate> armor_plates;
        if (bounding_boxes.size() >= 2) {
            for (size_t i = 0; i < bounding_boxes.size() - 1; i++) {
                RotatedRect first = bounding_boxes[i];
                RotatedRect second = bounding_boxes[i+1];

                Point2f first_points[4];
                Point2f second_points[4];
                first.points(first_points);
                second.points(second_points);

                int tl_x, tl_y, br_x, br_y;
                float angle_first = normalize_0_180(first);
                if (angle_first > 90) {
                    tl_x = first_points[1].x;
                    tl_y = min(first_points[1].y, second_points[2].y);
                    br_x = second_points[3].x;
                    br_y = max(first_points[0].y, second_points[3].y);
                }
                else {
                    tl_x = first_points[3].x;
                    tl_y = min(first_points[1].y, second_points[2].y);
                    br_x = second_points[1].x;
                    br_y = max(first_points[0].y, second_points[3].y);
                }

                Point tl(tl_x, tl_y), br(br_x, br_y);
                int x = (tl.x + br.x) / 2;
                int y = (tl.y + br.y) / 2;

                armor_plates.push_back(ArmorPlate{tl, br, Point(x,y), first, second});
            }
        }
        return std::make_tuple(armor_plates, bounding_boxes);
    }

    vector<ArmorPlate> find_best_armor_plates(vector<ArmorPlate> armor_plates) {
        int scores[armor_plates.size()] = {0};

        // Check if armor plate light angles are within 5 degrees of each other
        for (size_t i = 0; i < armor_plates.size(); i++) {
            float angle_left = normalize_0_180(armor_plates.at(i).left_light);
            float angle_right = normalize_0_180(armor_plates.at(i).right_light);
            if (abs(angle_left - angle_right) < 5) {
                scores[i] += 1;
            }
        }

        // Check if armor plate light height differences are within 1/4 of light height
        for (size_t i = 0; i < armor_plates.size(); i++) {
            int left_light_y = armor_plates.at(i).left_light.boundingRect().tl().y;
            int right_light_y = armor_plates.at(i).right_light.boundingRect().tl().y;

            int avg_light_height = (armor_plates.at(i).left_light.boundingRect().height + armor_plates.at(i).right_light.boundingRect().height) / 2;

            if (abs(left_light_y - right_light_y) < avg_light_height / 4) {
                scores[i] += 1;
            }
        }

        // Check if armor plates is wider than it is tall
        for (size_t i = 0; i < armor_plates.size(); i++) {
            int height = armor_plates.at(i).br.y - armor_plates.at(i).tl.y;
            int width = armor_plates.at(i).br.x - armor_plates.at(i).tl.x;

            if (width > height) {
                scores[i] += 1;
            }
        }

        vector<ArmorPlate> best_armor_plates;
        bool finding_plates = true;
        while (finding_plates) {
            // Get next best score
            int max_score = -1;
            int max_index = -1;
            for (size_t i = 0; i < armor_plates.size(); i++) {
                if (scores[i] > max_score) {
                    max_score = scores[i];
                    max_index = i;
                }
            }

            // Remove neighbors and selected plate
            if (max_index > -1) {
                scores[max_index] = -1;
                if (max_index > 0) {
                    scores[max_index - 1] = -1;
                }
                if (max_index < armor_plates.size() - 1) {
                    scores[max_index + 1] = -1;
                }
                best_armor_plates.push_back(armor_plates.at(max_index));
            }

            // All plates found, exit
            else {
                finding_plates = false;
            }
        }

        return best_armor_plates;
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

        // auto [accepted_rects, contours] = find_bounding_boxes(color_mask);
        // auto [armor_plates, bounding_boxes] = find_armor_plates(accepted_rects);

        auto [accepted_rects, contours] = find_rotated_bounding_boxes(color_mask);
        auto [armor_plates, bounding_boxes] = find_rotated_armor_plates(accepted_rects);

        armor_plates = find_best_armor_plates(armor_plates);

        if (this->get_parameter("enable_debug").as_bool() == true && armor_plates.size() > 0) {
            for (size_t i = 0; i < bounding_boxes.size(); i++) // iterate through each contour.
            {
                // rectangle(image_all_bounded_boxes, bounding_boxes[i].tl(), bounding_boxes[i].br(), Scalar(0, 255, 0), 5); 
                Point2f points[4];
                bounding_boxes[i].points(points);
                for (size_t j = 0; j < 4; j++) {
                    line(image_all_bounded_boxes, points[j], points[(j+1)%4], Scalar(0, 255, 0), 5);
                }
            }

            for (size_t i = 0; i < armor_plates.size(); i++) {
                rectangle(image_all_bounded_boxes, armor_plates[i].tl, armor_plates[i].br, Scalar(255, 0, 0), 5);
                circle(image_all_bounded_boxes, armor_plates[i].center, 2, Scalar(0, 0, 255), 8);
            }

            // sensor_msgs::msg::Image msg = *cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", image_all_bounded_boxes).toImageMsg();
            std_msgs::msg::UInt16 depth_msg; 
            depth_msg.data = depth_img.at<uint16_t>(armor_plates[0].center);

            // image_result_->publish(msg);
            depth_result_->publish(depth_msg);
        }

        sensor_msgs::msg::Image msg = *cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", image_all_bounded_boxes).toImageMsg();
        image_result_->publish(msg);

        stampede_msgs::msg::ObjectLogInput output_msg = stampede_msgs::msg::ObjectLogInput();
        for (size_t i = 0; i < armor_plates.size(); i++) {
            stampede_msgs::msg::BoundingBox bbox = stampede_msgs::msg::BoundingBox();
            bbox.center_x = armor_plates[i].center.x;
            bbox.center_y = armor_plates[i].center.y;
            bbox.width = armor_plates[i].br.x - armor_plates[i].tl.x;
            bbox.height = armor_plates[i].br.y - armor_plates[i].br.y;
            bbox.depth = depth_img.at<uint16_t>(armor_plates[i].center);
            output_msg.boxes.push_back(bbox);
        }
        classical_output_->publish(output_msg);
    }
    rclcpp::Subscription<realsense2_camera_msgs::msg::RGBD>::SharedPtr subscription_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_result_;
    rclcpp::Publisher<std_msgs::msg::UInt16>::SharedPtr depth_result_;
    rclcpp::Publisher<stampede_msgs::msg::ObjectLogInput>::SharedPtr classical_output_;

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