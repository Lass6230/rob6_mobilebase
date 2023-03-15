#include <rclcpp/rclcpp.hpp>
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/transform_broadcaster.h"

class Sensor: public rclcpp::Node{
private:
    void objectCallback(){

        geometry_msgs::msg::TransformStamped t;

        // Read message content and assign it to
        // corresponding tf variables
        t.header.stamp = this->get_clock()->now();
        t.header.frame_id = "tool";
        t.child_frame_id = "object";

        // Turtle only exists in 2D, thus we get x and y translation
        // coordinates from the message and set the z coordinate to 0
        t.transform.translation.x = 0.05;
        t.transform.translation.y = 0.0;
        t.transform.translation.z = 0.0;

        // For the same reason, turtle can only rotate around one axis
        // and this why we set rotation in x and y to 0 and obtain
        // rotation in z axis from the message
        tf2::Quaternion q;
        q.setRPY(0, 0, 0);
        t.transform.rotation.x = q.x();
        t.transform.rotation.y = q.y();
        t.transform.rotation.z = q.z();
        t.transform.rotation.w = q.w();

        // Send the transformation
        tf_broadcaster_->sendTransform(t);
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
    }

    // Timer for the simulated detected object
    rclcpp::TimerBase::SharedPtr object_timer_;
    // Publisher for the simulated detected object
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;
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
    }

    ~Sensor(){}
};


int main(int argc, char const *argv[]){
    rclcpp::init(argc, argv);
    auto node = std::make_shared<Sensor>("sensor");
    rclcpp::spin(node);
    return 0;
}