#include <rclcpp/rclcpp.hpp>
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/transform_broadcaster.h"
#include "std_msgs/msg/int8.hpp"
class Sensor: public rclcpp::Node{
private:
    void objectCallback(){

        

        // Read message content and assign it to
        // corresponding tf variables
        
        // All transforms must be correctly timestamped
        pose_.header.stamp = this->get_clock()->now();
        pose_.header.frame_id = "tool_link";

        // if (count_ % 2) {
        //     pose_.pose.position.x = 1.0;
        //     pose_.pose.position.y = 1.0;
        // } else {
        //     pose_.pose.position.x = 2.0;
        //     pose_.pose.position.y = 3.0;
        // }

        pose_.pose.position.x = 0.06;

        //pose_.pose.position.y = 0.0;
        pose_.pose.position.z = 0.03;
        // Change the detected object's position, depending on whether count_ is even or odd
        count_++;
        pose_pub_->publish(pose_);

        geometry_msgs::msg::TransformStamped t;

        t.header.stamp = this->get_clock()->now();
        t.header.frame_id = "camera_link";
        t.child_frame_id = "aruco";

        t.transform.translation.x = 0.1;//atof(transformation[2]);
        //t.transform.translation.y = atof(transformation[3]);
        //t.transform.translation.z = atof(transformation[4]);
        tf2::Quaternion q;
        q.setRPY(0,0,0);
        t.transform.rotation.x = q.x();
        t.transform.rotation.y = q.y();
        t.transform.rotation.z = q.z();
        t.transform.rotation.w = q.w();

        tf_broadcaster_->sendTransform(t);
        geometry_msgs::msg::TransformStamped t2;
        t2.header.stamp = this->get_clock()->now();
        t2.header.frame_id = "camera_link";
        t2.child_frame_id = "ball";

        t2.transform.translation.x = 0.33;//atof(transformation[2]);
        t2.transform.translation.y = 0.1;//atof(transformation[3]);
        t2.transform.translation.z = -0.09;//atof(transformation[4]);
        tf2::Quaternion q2;
        q2.setRPY(0,0,0);
        t2.transform.rotation.x = q.x();
        t2.transform.rotation.y = q.y();
        t2.transform.rotation.z = q.z();
        t2.transform.rotation.w = q.w();

        tf_broadcaster_->sendTransform(t2);

        status_msg.data = 1;
        status_ball_->publish(status_msg);

    }

    // Timer for the simulated detected object
    rclcpp::TimerBase::SharedPtr object_timer_;
    // Publisher for the simulated detected object
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;
    rclcpp::Publisher<std_msgs::msg::Int8>::SharedPtr status_ball_;
    std_msgs::msg::Int8 status_msg;
    // Pose to publish
    geometry_msgs::msg::PoseStamped pose_;
    // Aux variable that determines the detected object's position

    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    int count_ = 0;
    
public:
    Sensor(const std::string & name):Node(name){

        tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
        // Initialize timer for object detection
        object_timer_ = create_wall_timer(std::chrono::seconds(1), 
            std::bind(&Sensor::objectCallback, this));

        // Initialize publisher for object detection
        pose_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>(
            "/detected_object", 10);
        status_ball_ = create_publisher<std_msgs::msg::Int8>("/status_ball", 10);
    }

    ~Sensor(){}
};


int main(int argc, char const *argv[]){
    rclcpp::init(argc, argv);
    auto node = std::make_shared<Sensor>("sensor");
    rclcpp::spin(node);
    return 0;
}