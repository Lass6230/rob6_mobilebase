#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int64_multi_array.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <geometry_msgs/msg/pose.hpp>
#include "iostream"


class OdometryCalculator : public rclcpp::Node
{
public:
  OdometryCalculator() : Node("odometry_calculator")
  {
    encoder_ticks_left_ = 0;
    encoder_ticks_right_ = 0;
    last_encoder_ticks_left_ = 0;
    last_encoder_ticks_right_ = 0;
    last_time_ = this->now();

    // Create a subscription to the encoder ticks topic
    encoder_sub_ = this->create_subscription<std_msgs::msg::Int64MultiArray>(
      "/encoder1", 1,
      std::bind(&OdometryCalculator::encoderCallback, this, std::placeholders::_1));

      

    // Create a publisher for the odometry topic
    odometry_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("/odom", 10);
  }

private:
  void encoderCallback(const std_msgs::msg::Int64MultiArray::SharedPtr msg)
  {
    encoder_ticks_left_ = msg->data[0] >> 16;  
    encoder_ticks_right_ = msg->data[1] & 0xFFFF;  
    std::cout << encoder_ticks_left_ + "\n";

    // Calculate time since last callback and update last time
    rclcpp::Time current_time = this->now();
    rclcpp::Duration dt = current_time - last_time_;
    last_time_ = current_time;

    // Calculate wheel displacements and average them
    double displacement_left = (encoder_ticks_left_ - last_encoder_ticks_left_) * TICKS_TO_DISTANCE_;
    double displacement_right = (encoder_ticks_right_ - last_encoder_ticks_right_) * TICKS_TO_DISTANCE_;
    double displacement = (displacement_left + displacement_right) / 2.0;

    // Calculate robot heading change and average it
    double heading_change = (displacement_right - displacement_left) / WHEELBASE_;
    double heading = last_heading_ + heading_change / 2.0;

    // Calculate robot position change
    double dx = displacement * cos(heading);
    double dy = displacement * sin(heading);

    // Update robot position and heading
    x_ += dx;
    y_ += dy;
    last_heading_ = heading;

    // Create and publish the odometry message
    nav_msgs::msg::Odometry odom_msg;
    odom_msg.header.stamp = current_time;
    odom_msg.header.frame_id = "odom";
    odom_msg.child_frame_id = "base_link";
    odom_msg.pose.pose.position.x = x_;
    odom_msg.pose.pose.position.y = y_;
    odom_msg.pose.pose.position.z = 0.0;
    odom_msg.pose.pose.orientation = tf2::toMsg(tf2::Quaternion(tf2::Vector3(0, 0, 1), heading));
    
    odom_msg.twist.twist.linear.x = displacement / dt.seconds();
    odom_msg.twist.twist.angular.z = heading_change / dt.seconds();
    odometry_pub_->publish(odom_msg);

    // Update last encoder tick counts
    last_encoder_ticks_left_ = encoder_ticks_left_;
    last_encoder_ticks_right_ = encoder_ticks_right_;
  }


  // ROS 2 objects
  rclcpp::Subscription<std_msgs::msg::Int64MultiArray>::SharedPtr encoder_sub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odometry_pub_;

  // Odometry state variables
  double x_ = 0.0;
  double y_ = 0.0;
  double last_heading_ = 0.0;

  // Encoder state variables
  int64_t encoder_ticks_left_;
  int64_t encoder_ticks_right_;
  int64_t last_encoder_ticks_left_;
  int64_t last_encoder_ticks_right_;
  rclcpp::Time last_time_;

  // Constants
  const double TICKS_TO_DISTANCE_ = 0.000009446;  // Distance per encoder tick in meters
  const double WHEELBASE_ = 0.271;  // Distance between the wheels in meters
};


int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<OdometryCalculator>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}

