#include <cstdio>
#include <rclcpp/rclcpp.hpp>
#include "std_msgs/msg/int32.hpp"
#include "std_msgs/msg/int32_multi_array.hpp"
#include <vision_msgs/msg/detection3_d_array.hpp>
#include <vision_msgs/msg/detection3_d.hpp>
#include "std_msgs/msg/float32_multi_array.hpp"
#include <opencv2/core.hpp>

// at what health should the robot retreat
#define HEALTH_THRESHOLD 35

class AutonavNode : public rclcpp::Node {
public:
    AutonavNode() : Node("AutonavNode") {
        /*Examples of the types of subscriptions needed for the autonavigation behaviors.*/
        localization_ = this->create_subscription<vision_msgs::msg::Detection3D>("localization_topic", 10, 
                        std::bind(&AutonavNode::localization_callback, this, std::placeholders::_1));

        odometry_ = this->create_subscription<std_msgs::msg::Float32MultiArray>("odometry_topic", 10, 
                        std::bind(&AutonavNode::odometry_callback, this, std::placeholders::_1));

        robots_ = this->create_subscription<vision_msgs::msg::Detection3DArray>("robots_topic", 10, 
                        std::bind(&AutonavNode::odometry_callback, this, std::placeholders::_1));

        //Someone check me on this plz - Daniel
        target_pub_ = this->create_publisher<std_msgs::msg::Float32MultiArray>("target_topic", 10);

        //TODO: make shoot pub - Daniel

        timer_ = this->create_wall_timer(std::chrono::seconds(1), std::bind(&AutonavNode::update_state_machine, this));
    }

private:
    void localization_callback(const vision_msgs::msg::Detection3D::SharedPtr msg) {
        cv::Point3f localizaiton_pos;
        localizaiton_pos.x = msg->bbox.center.position.x;
        localizaiton_pos.y = msg->bbox.center.position.y;
        localizaiton_pos.z = msg->bbox.center.position.z;
        RCLCPP_INFO(this->get_logger(), "Received: x = %d, y = %d, z = %d", localizaiton_pos.x, localizaiton_pos.y, localizaiton_pos.z);
    }

    void odometry_callback(const std_msgs::msg::Float32MultiArray::SharedPtr msg) {
        odometry_pos = msg->data;
        RCLCPP_INFO(this->get_logger(), 
                "Received: x = %d, y = %d, z = %d, chassis_pitch = %d, chassis_yaw = %d, chassis_roll = %d, turret_pitch = %d, turret_yaw = %d", 
                odometry_pos[0], odometry_pos[1], odometry_pos[2], odometry_pos[3], odometry_pos[4], odometry_pos[5], odometry_pos[6], odometry_pos[7]);
    }

    void robots_callback(const vision_msgs::msg::Detection3DArray::SharedPtr msg) {        
        // converting detections msg data into points in the robots vector
        for (const auto &robot : msg->detections) {
            cv::Point3f point;
            point.x = robot.bbox.center.position.x;
            point.y = robot.bbox.center.position.y;
            point.z = robot.bbox.center.position.z;
            robots.push_back(point);
            RCLCPP_INFO(this->get_logger(), "Robot at: x = %d, y = %d, z = %d", point.x, point.y, point.z);
        }
    }

    void health_callback(const std_msgs::msg::Int32::SharedPtr msg) {
        health = msg->data;
        RCLCPP_INFO(this->get_logger(), "Recieved: heath = %d", health);
    }

    void update_state_machine() {
        /*If statement to Pick Behavior*/

        if (health == 0){
            // retreat with out shooting 
        }

        if (health <= HEALTH_THRESHOLD ) {
            // retreat
        }

        // patrol needs second condition checking that there are enemies on the point
        /*
            Required input:
            1)Location
            2)Health
            3)Ammo
            4)Detection

            Required output:
            1) State?
            2) Target pose?
            3) Target aim?

            Design questions:
            1) Where patrol? For now, stand still
            2) How scan (camera/chassis movement)? For now, moving cam with target
            3) When shoot? For now, on sight
            4) Chase enemies? For now, only trace but not chase
            5) Stay still or zigzag? For now, stay still
            6) Take cover? For now, no
            7) Focus/split fire? For now, focus on first target
            Current design just stands still and focuses fire on first tgt
        */
        bool split_fire = false;
        if (health > HEALTH_THRESHOLD && robots.size() > 0){
            RCLCPP_INFO(this->get_logger(), "Current State: Patrol");
            //Aim and shoot at robot, think someone else is working on code for this

            if(split_fire){
                //Split fire between two tgts, need messages for two turrets first
            }
        }
        
        // capturing the point needs second condition checking that there are no enemies on the point
        if (health > HEALTH_THRESHOLD && robots.size() == 0){
            // capture
        }

        /*Publish the goal posisiton, cost map, and anything else thats relevant to any other system here.*/
    }

    // subscriptions. Assume localization and odemetry are subscription datatypes are correct. 
    rclcpp::Subscription<vision_msgs::msg::Detection3D>::SharedPtr localization_;
    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr odometry_;
    rclcpp::Subscription<vision_msgs::msg::Detection3DArray>::SharedPtr robots_;

    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr target_pub_; //Target publisher - Daniel
    rclcpp::TimerBase::SharedPtr timer_;
    std::vector<cv::Point3f> robots; // a vector containing point3fs of detection data. each entry is a detection data
    std::vector<float> odometry_pos; // a vector of floats, containing position of the robot, x,y,z, and all turret/chassis data
    cv::Point3f localizaiton_pos; // a 3d point of the position of the robot, gotten from the localization data
    int health; // health of the robot

};



/*Put all behabiors in methods here! Call them in the if statement inside update_state_machine, which picks a behavior
  based on preconditions.*/




int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<AutonavNode>());
  rclcpp::shutdown();
  return 0;
}
