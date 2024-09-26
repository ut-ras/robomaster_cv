/** 
 * References:
 * https://github.com/ut-ras/robomaster_cv/wiki/Onboarding
 * http://wiki.ros.org/cv_bridge/Tutorials/UsingCvBridgeToConvertBetweenROSImagesAndOpenCVImages
 * https://docs.opencv.org/4.x/d4/da8/group__imgcodecs.html#ga8ac397bd09e48851665edbe12aa28f25
 * https://docs.opencv.org/4.x/d4/d61/tutorial_warp_affine.html
*/ 

#include <cstdio>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>

using namespace cv;

class PrajithNode : public rclcpp::Node {
  public:
    PrajithNode() : Node("prajithv_node") {
      sub_ = this->create_subscription<sensor_msgs::msg::Image>(
        "image_raw",
        rclcpp::QoS(1),
        std::bind(&PrajithNode::prajithvCallback, this, std::placeholders::_1)
      );
    }

    void prajithvCallback(sensor_msgs::msg::Image::SharedPtr msg) {
      cv_bridge::CvImagePtr cv_image_ptr = cv_bridge::toCvCopy(msg);
      Mat img = cv_image_ptr->image;

      Point2f srcTri[3];
      srcTri[0] = Point2f(0.f, 0.f);
      srcTri[1] = Point2f(img.cols - 1.f, 0.f);
      srcTri[2] = Point2f(0.f, img.rows - 1.f);

      Point2f dstTri[3];
      dstTri[0] = Point2f(0.f, img.rows * 0.5f);
      dstTri[1] = Point2f(img.cols * 0.75f, 0.f);
      dstTri[2] = Point2f(img.cols * 0.25f, img.rows - 1.f);

      Mat warp_mat = getAffineTransform(srcTri, dstTri);
      Mat warp_dst = Mat::zeros(img.rows, img.cols, img.type());

      warpAffine(img, warp_dst, warp_mat, warp_dst.size());

      imwrite("warped_image.jpg", warp_dst);
    }

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_;
};

int main(int argc, char ** argv)
{
  (void) argc;
  (void) argv;

  rclcpp::init(argc, argv);

  auto node = std::make_shared<PrajithNode>();

  RCLCPP_INFO(node->get_logger(), "prajithv node starting");

  rclcpp::spin(node);
  rclcpp::shutdown();

  return 0;
}
