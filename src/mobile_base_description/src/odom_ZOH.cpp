#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include <iostream>

class OdomToTfNode : public rclcpp::Node
{
public:
  u_int32_t publishes = 0;
  bool publishing_odometry = false;
  OdomToTfNode()
  : Node("odom_to_tf"), tf_broadcaster_(this)
  {
    // Create a subscriber to the /odom topic
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      "odom", 10, std::bind(&OdomToTfNode::odom_callback, this, std::placeholders::_1));
    // Create a timer to publish the transform at a fixed frequency
    tf_timer_ = create_wall_timer(std::chrono::milliseconds(30), std::bind(&OdomToTfNode::tf_callback, this));
  }

private:
  void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    // Store the latest odometry message
    odom_ = msg;
    //std::cout << "new odom!\n";
    publishing_odometry = false;
    publishes +=1;

    if (publishes%100 == 0) {
      std::cout << publishes << "\n";
    }
    
  }

  void tf_callback()
  {
    
    if (odom_)
    {
      if(!publishing_odometry)
      {
        //std::cout << "publishing old odom\n";
        publishing_odometry = true;
      }
      
      // Create a transform from the odometry message
      geometry_msgs::msg::TransformStamped tf;
      tf.header.stamp = odom_->header.stamp;
      //tf.header.frame_id = odom_->header.frame_id;
      //tf.child_frame_id = odom_->child_frame_id;
      tf.header.frame_id = "odom";
      tf.child_frame_id = "base_link";
      tf.transform.translation.x = odom_->pose.pose.position.x;
      tf.transform.translation.y = odom_->pose.pose.position.y;
      tf.transform.translation.z = odom_->pose.pose.position.z;
      tf.transform.rotation = odom_->pose.pose.orientation;

      // Publish the transform
      tf_broadcaster_.sendTransform(tf);
    }
    
  }

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::TimerBase::SharedPtr tf_timer_;
  nav_msgs::msg::Odometry::ConstSharedPtr odom_;
  tf2_ros::TransformBroadcaster tf_broadcaster_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<OdomToTfNode>());
  rclcpp::shutdown();
  return 0;
}
