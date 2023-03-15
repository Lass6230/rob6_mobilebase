#include "rclcpp/rclcpp.hpp"
<<<<<<< HEAD
#include "std_msgs/msg/int64_multi_array.hpp"
=======
#include "std_msgs/msg/int32.hpp"
#include "std_msgs/msg/int64.hpp"
>>>>>>> galactic
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <geometry_msgs/msg/pose.hpp>
<<<<<<< HEAD
#include "iostream"
=======
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/time_synchronizer.h>
#include <math.h>
>>>>>>> galactic

using std::placeholders::_1;
using std::placeholders::_2;

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
<<<<<<< HEAD
    encoder_sub_ = this->create_subscription<std_msgs::msg::Int64MultiArray>(
      "/encoder1", 1,
      std::bind(&OdometryCalculator::encoderCallback, this, std::placeholders::_1));
=======
    
    sub1_.subscribe(this,"/encoder2");
    sub2_.subscribe(this,"/encoder1");

    sync_ = std::make_shared<message_filters::TimeSynchronizer<std_msgs::msg::Int64, std_msgs::msg::Int64>>(sub1_, sub2_, 3);
    sync_->registerCallback(std::bind(&OdometryCalculator::encoderCallback, this, _1, _2));
  

   
    // encoder1_sub_ = this->create_subscription<std_msgs::msg::Int64>(
    //   "encoder1", 10,
    //   std::bind(&OdometryCalculator::encoderCallback, this, std::placeholders::_1));
    // encoder2_sub_ = this->create_subscription<std_msgs::msg::Int64>(
    //   "encoder2", 10,
    //   std::bind(&OdometryCalculator::encoderCallback, this, std::placeholders::_1));
>>>>>>> galactic

      

    // Create a publisher for the odometry topic
    odometry_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("/odom", 10);
  }

private:
<<<<<<< HEAD
  void encoderCallback(const std_msgs::msg::Int64MultiArray::SharedPtr msg)
  {
    encoder_ticks_left_ = msg->data[0] >> 16;  
    encoder_ticks_right_ = msg->data[1] & 0xFFFF;  
    std::cout << encoder_ticks_left_ + "\n";
=======
  void encoderCallback(const std_msgs::msg::Int64::ConstSharedPtr msg1, std_msgs::msg::Int64::ConstSharedPtr msg2)
  { 
    
    encoder_ticks_left_ = msg1->data >> 64;  
    encoder_ticks_right_ = msg2->data & 0xFFFFFFFFFFFFFFFF;  
>>>>>>> galactic

    // Calculate time since last callback and update last time
    rclcpp::Time current_time = this->now();
    rclcpp::Duration dt = current_time - last_time_;
    last_time_ = current_time;


    // Calculate wheel displacements and average them
    double displacement_left = (encoder_ticks_left_ - last_encoder_ticks_left_) * TICKS_TO_DISTANCE_;
    double displacement_right = (encoder_ticks_right_ - last_encoder_ticks_right_) * TICKS_TO_DISTANCE_;
    double displacement = (displacement_left + displacement_right) / 2.0;

    int64_t diff_enc1 = encoder_ticks_left_ - last_encoder_ticks_left_;
    int64_t diff_enc2 = encoder_ticks_right_ - last_encoder_ticks_right_;
    double d_theta;
    if(diff_enc1 == diff_enc2){
      d_theta = 0.0;
      current_x = current_x + (displacement * cos(current_theta));
      current_y = current_y + (displacement * sin(current_theta));
    }
    else{
      d_theta = (displacement_right -displacement_left)/ WHEELBASE_;
      double r = displacement/d_theta;
      
      current_x = current_x + (r * (sin(d_theta + current_theta) - sin(current_theta)));
      current_y = current_y + (r * (cos(d_theta + current_theta) - cos(current_theta)));

      current_theta = OdometryCalculator::constrainAngle1(current_theta + d_theta);
      //current_theta = current_theta + d_theta;
      
    }

   

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
    odom_msg.pose.pose.position.x = current_x;
    odom_msg.pose.pose.position.y = current_y;
    odom_msg.pose.pose.position.z = 0.0;
    odom_msg.pose.pose.orientation = tf2::toMsg(tf2::Quaternion(tf2::Vector3(0, 0, 1), current_theta));
    
    odom_msg.twist.twist.linear.x = displacement / dt.seconds();
    odom_msg.twist.twist.angular.z = d_theta / dt.seconds();
    odometry_pub_->publish(odom_msg);

    // Update last encoder tick counts
    last_encoder_ticks_left_ = encoder_ticks_left_;
    last_encoder_ticks_right_ = encoder_ticks_right_;
  }

  double constrainAngle1(double x){
    x = fmod(x + M_PI,2*M_PI);
    if (x < 0)
        x += 2*M_PI;
    return x - M_PI;
}

double constrainAngle(double x){
    x = fmod(x,2*M_PI);
    if (x < 0)
        x += 2*M_PI;
    return x;
}
  // ROS 2 objects
<<<<<<< HEAD
  rclcpp::Subscription<std_msgs::msg::Int64MultiArray>::SharedPtr encoder_sub_;
=======
  message_filters::Subscriber<std_msgs::msg::Int64> sub1_;
  message_filters::Subscriber<std_msgs::msg::Int64> sub2_;
  std::shared_ptr<message_filters::TimeSynchronizer<std_msgs::msg::Int64, std_msgs::msg::Int64>> sync_;
  rclcpp::Subscription<std_msgs::msg::Int64>::SharedPtr encoder1_sub_;
  rclcpp::Subscription<std_msgs::msg::Int64>::SharedPtr encoder2_sub_;
>>>>>>> galactic
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odometry_pub_;

  // Odometry state variables
  double x_ = 0.0;
  double y_ = 0.0;
  double last_heading_ = 0.0;

  double current_x = 0.0;
  double current_y = 0.0;
  double current_z = 0.0;
  double current_theta = 0.0;

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

