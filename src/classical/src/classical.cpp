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
#include "stampede_msgs/msg/turret_data.hpp"
#include "stampede_msgs/msg/uart.hpp"

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
        cv_output_ = this->create_publisher<stampede_msgs::msg::Uart>("data_tx", 10);
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

    void send_cv_data(const stampede_msgs::msg::TurretData &data) {
        stampede_msgs::msg::Uart cv_output_msg = stampede_msgs::msg::Uart();
        cv_output_msg.frame_head_byte = 0xA5;
        cv_output_msg.frame_data_length = static_cast<uint16_t>(sizeof(turret_data_msg));
        cv_output_msg.frame_sequence = 0x00;
        uint8_t header[4] = {cv_output_msg.frame_head_byte, (uint8_t) (cv_output_msg.frame_data_length), (uint8_t) (cv_output_msg.frame_data_length >> 8), cv_output_msg.frame_sequence};
        cv_output_msg.frame_crc8 = Get_CRC8_Check_Sum(header, 4, CRC8_INIT);

        cv_output_msg.msg_type = 0x000F;
        cv_output_msg.data = data;
        uint8_t msg_arr[7 + cv_output_msg.frame_data_length];
        memcpy(msg_arr, header, 4);
        msg_arr[4] = cv_output_msg.frame_crc8;
        msg_arr[5] = cv_output_msg.msg_type & 0xFF;
        msg_arr[6] = cv_output_msg.msg_type >> 8;
        memcpy(&(msg_arr[7]), &cv_output_msg.data, cv_output_msg.frame_data_length);
        cv_output_msg.crc16 = Get_CRC16_Check_Sum(msg_arr, sizeof(msg_arr), CRC_INIT);

        cv_output->publish(cv_output_msg);
    }

    //crc8 generator polynomial:G(x)=x8+x5+x4+1
    const unsigned char CRC8_INIT = 0xff;
    const unsigned char CRC8_TAB[256] = {
        0x00, 0x5e, 0xbc, 0xe2, 0x61, 0x3f, 0xdd, 0x83, 0xc2, 0x9c, 0x7e, 0x20, 0xa3, 0xfd, 0x1f, 0x41,
        0x9d, 0xc3, 0x21, 0x7f, 0xfc, 0xa2, 0x40, 0x1e, 0x5f, 0x01, 0xe3, 0xbd, 0x3e, 0x60, 0x82, 0xdc, 0x23, 0x7d, 0x9f,
        0xc1, 0x42, 0x1c, 0xfe, 0xa0, 0xe1, 0xbf, 0x5d, 0x03, 0x80, 0xde, 0x3c, 0x62, 0xbe, 0xe0, 0x02, 0x5c, 0xdf, 0x81,
        0x63, 0x3d, 0x7c, 0x22, 0xc0, 0x9e, 0x1d, 0x43, 0xa1, 0xff, 0x46, 0x18, 0xfa, 0xa4, 0x27, 0x79, 0x9b, 0xc5, 0x84,
        0xda, 0x38, 0x66, 0xe5, 0xbb, 0x59, 0x07, 0xdb, 0x85, 0x67, 0x39, 0xba, 0xe4, 0x06, 0x58, 0x19, 0x47, 0xa5,
        0xfb, 0x78, 0x26, 0xc4, 0x9a, 0x65, 0x3b, 0xd9, 0x87, 0x04, 0x5a, 0xb8, 0xe6, 0xa7, 0xf9, 0x1b, 0x45, 0xc6,
        0x98, 0x7a, 0x24, 0xf8, 0xa6, 0x44, 0x1a, 0x99, 0xc7, 0x25, 0x7b, 0x3a, 0x64, 0x86, 0xd8, 0x5b, 0x05, 0xe7,
        0xb9,
        0x8c, 0xd2, 0x30, 0x6e, 0xed, 0xb3, 0x51, 0x0f, 0x4e, 0x10, 0xf2, 0xac, 0x2f, 0x71, 0x93, 0xcd, 0x11, 0x4f, 0xad,
        0xf3, 0x70, 0x2e, 0xcc, 0x92, 0xd3, 0x8d, 0x6f, 0x31, 0xb2, 0xec, 0x0e, 0x50, 0xaf, 0xf1, 0x13, 0x4d, 0xce, 0x90,
        0x72, 0x2c, 0x6d, 0x33, 0xd1, 0x8f, 0x0c, 0x52, 0xb0, 0xee, 0x32, 0x6c, 0x8e, 0xd0, 0x53, 0x0d, 0xef, 0xb1, 0xf0,
        0xae, 0x4c, 0x12, 0x91, 0xcf, 0x2d, 0x73, 0xca, 0x94, 0x76, 0x28, 0xab, 0xf5, 0x17, 0x49, 0x08, 0x56, 0xb4, 0xea,
        0x69, 0x37, 0xd5, 0x8b, 0x57, 0x09, 0xeb, 0xb5, 0x36, 0x68, 0x8a, 0xd4, 0x95, 0xcb, 0x29, 0x77, 0xf4, 0xaa,
        0x48, 0x16, 0xe9, 0xb7, 0x55, 0x0b, 0x88, 0xd6, 0x34, 0x6a, 0x2b, 0x75, 0x97, 0xc9, 0x4a, 0x14, 0xf6, 0xa8,
        0x74, 0x2a, 0xc8, 0x96, 0x15, 0x4b, 0xa9, 0xf7, 0xb6, 0xe8, 0x0a, 0x54, 0xd7, 0x89, 0x6b, 0x35,
    };

    unsigned char Get_CRC8_Check_Sum(unsigned char *pchMessage, unsigned int dwLength, unsigned char ucCRC8) {
        unsigned char ucIndex;
        while (dwLength--) {
            ucIndex = ucCRC8^(*pchMessage++);
            ucCRC8 = CRC8_TAB[ucIndex];
        }
        
        return(ucCRC8);
    }

    uint16_t CRC_INIT = 0xffff;
    const uint16_t wCRC_Table[256] = {
        0x0000, 0x1189, 0x2312, 0x329b, 0x4624, 0x57ad, 0x6536, 0x74bf,
        0x8c48, 0x9dc1, 0xaf5a, 0xbed3, 0xca6c, 0xdbe5, 0xe97e, 0xf8f7,
        0x1081, 0x0108, 0x3393, 0x221a, 0x56a5, 0x472c, 0x75b7, 0x643e,
        0x9cc9, 0x8d40, 0xbfdb, 0xae52, 0xdaed, 0xcb64, 0xf9ff, 0xe876,
        0x2102, 0x308b, 0x0210, 0x1399, 0x6726, 0x76af, 0x4434, 0x55bd,
        0xad4a, 0xbcc3, 0x8e58, 0x9fd1, 0xeb6e, 0xfae7, 0xc87c, 0xd9f5,
        0x3183, 0x200a, 0x1291, 0x0318, 0x77a7, 0x662e, 0x54b5, 0x453c,
        0xbdcb, 0xac42, 0x9ed9, 0x8f50, 0xfbef, 0xea66, 0xd8fd, 0xc974,
        0x4204, 0x538d, 0x6116, 0x709f, 0x0420, 0x15a9, 0x2732, 0x36bb,
        0xce4c, 0xdfc5, 0xed5e, 0xfcd7, 0x8868, 0x99e1, 0xab7a, 0xbaf3,
        0x5285, 0x430c, 0x7197, 0x601e, 0x14a1, 0x0528, 0x37b3, 0x263a,
        0xdecd, 0xcf44, 0xfddf, 0xec56, 0x98e9, 0x8960, 0xbbfb, 0xaa72,
        0x6306, 0x728f, 0x4014, 0x519d, 0x2522, 0x34ab, 0x0630, 0x17b9,
        0xef4e, 0xfec7, 0xcc5c, 0xddd5, 0xa96a, 0xb8e3, 0x8a78, 0x9bf1,
        0x7387, 0x620e, 0x5095, 0x411c, 0x35a3, 0x242a, 0x16b1, 0x0738,
        0xffcf, 0xee46, 0xdcdd, 0xcd54, 0xb9eb, 0xa862, 0x9af9, 0x8b70,
        0x8408, 0x9581, 0xa71a, 0xb693, 0xc22c, 0xd3a5, 0xe13e, 0xf0b7,
        0x0840, 0x19c9, 0x2b52, 0x3adb, 0x4e64, 0x5fed, 0x6d76, 0x7cff,
        0x9489, 0x8500, 0xb79b, 0xa612, 0xd2ad, 0xc324, 0xf1bf, 0xe036,
        0x18c1, 0x0948, 0x3bd3, 0x2a5a, 0x5ee5, 0x4f6c, 0x7df7, 0x6c7e,
        0xa50a, 0xb483, 0x8618, 0x9791, 0xe32e, 0xf2a7, 0xc03c, 0xd1b5,
        0x2942, 0x38cb, 0x0a50, 0x1bd9, 0x6f66, 0x7eef, 0x4c74, 0x5dfd,
        0xb58b, 0xa402, 0x9699, 0x8710, 0xf3af, 0xe226, 0xd0bd, 0xc134,
        0x39c3, 0x284a, 0x1ad1, 0x0b58, 0x7fe7, 0x6e6e, 0x5cf5, 0x4d7c,
        0xc60c, 0xd785, 0xe51e, 0xf497, 0x8028, 0x91a1, 0xa33a, 0xb2b3,
        0x4a44, 0x5bcd, 0x6956, 0x78df, 0x0c60, 0x1de9, 0x2f72, 0x3efb,
        0xd68d, 0xc704, 0xf59f, 0xe416, 0x90a9, 0x8120, 0xb3bb, 0xa232,
        0x5ac5, 0x4b4c, 0x79d7, 0x685e, 0x1ce1, 0x0d68, 0x3ff3, 0x2e7a,
        0xe70e, 0xf687, 0xc41c, 0xd595, 0xa12a, 0xb0a3, 0x8238, 0x93b1,
        0x6b46, 0x7acf, 0x4854, 0x59dd, 0x2d62, 0x3ceb, 0x0e70, 0x1ff9,
        0xf78f, 0xe606, 0xd49d, 0xc514, 0xb1ab, 0xa022, 0x92b9, 0x8330,
        0x7bc7, 0x6a4e, 0x58d5, 0x495c, 0x3de3, 0x2c6a, 0x1ef1, 0x0f78
    };

    /*
    ** Descriptions: CRC16 checksum function
    ** Input: Data to check,Stream length, initialized checksum
    ** Output: CRC checksum
    */
    uint16_t Get_CRC16_Check_Sum(uint8_t *pchMessage, uint32_t dwLength, uint16_t wCRC) {
        uint8_t chData;
        if (pchMessage == NULL) {
            return 0xFFFF;
        }
        while(dwLength--) {
            chData = *pchMessage++;
            (wCRC) = ((uint16_t)(wCRC) >> 8) ^ wCRC_Table[((uint16_t)(wCRC) ^ (uint16_t)(chData)) & 0x00ff];
        }
        return wCRC;
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

        // stampede_msgs::msg::ObjectLogInput output_msg = stampede_msgs::msg::ObjectLogInput();
        // for (size_t i = 0; i < armor_plates.size(); i++) {
        //     stampede_msgs::msg::BoundingBox bbox = stampede_msgs::msg::BoundingBox();
        //     bbox.center_x = armor_plates[i].center.x;
        //     bbox.center_y = armor_plates[i].center.y;
        //     bbox.width = armor_plates[i].br.x - armor_plates[i].tl.x;
        //     bbox.height = armor_plates[i].br.y - armor_plates[i].tl.y;
        //     bbox.depth = depth_img.at<uint16_t>(armor_plates[i].center);
        //     output_msg.boxes.push_back(bbox);
        // }
        // classical_output_->publish(output_msg);

        stampede_msgs::msg::TurretData turret_data_msg = stampede_msgs::msg::TurretData();
        if (armor_plates.size == 0) {
            turret_data_msg.xpos = 0.0f;
            turret_data_msg.ypos = 0.0f;
            turret_data_msg.zpos = 0.0f;
            turret_data_msg.xvel = 0.0f;
            turret_data_msg.yvel = 0.0f;
            turret_data_msg.zvel = 0.0f;
            turret_data_msg.xacc = 0.0f;
            turret_data_msg.yacc = 0.0f;
            turret_data_msg.zacc = 0.0f;
            turret_data_msg.has_target = false;
        }

        else {
            turret_data_msg.xpos = armor_plates[0].center.x;
            turret_data_msg.ypos = armor_plates[0].center.y;
            turret_data_msg.zpos = depth_img.at<uint16_t>(armor_plates[i].center);
            turret_data_msg.xvel = 0.0f;
            turret_data_msg.yvel = 0.0f;
            turret_data_msg.zvel = 0.0f;
            turret_data_msg.xacc = 0.0f;
            turret_data_msg.yacc = 0.0f;
            turret_data_msg.zacc = 0.0f;
            turret_data_msg.has_target = true;
        }

        send_cv_data(*turret_data_msg);
    }
    rclcpp::Subscription<realsense2_camera_msgs::msg::RGBD>::SharedPtr subscription_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_result_;
    rclcpp::Publisher<std_msgs::msg::UInt16>::SharedPtr depth_result_;
    rclcpp::Publisher<stampede_msgs::msg::ObjectLogInput>::SharedPtr classical_output_;
    rclcpp::Publisher<stampede_msgs::msg::TurretData>::SharedPtr cv_output;

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