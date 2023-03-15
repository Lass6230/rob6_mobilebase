#define BOOST_BIND_NO_PLACEHOLDERS

#include "rclcpp/rclcpp.hpp"
//#include "crust_msgs/srv/RobotCmdSrv.hpp"
#include <crust_msgs/srv/robot_cmd_srv.hpp>
#include <memory>

#include <moveit/move_group_interface/move_group_interface.h>

#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/msg/display_robot_state.hpp>
#include <moveit_msgs/msg/display_trajectory.hpp>

#include <moveit_msgs/msg/attached_collision_object.hpp>
#include <moveit_msgs/msg/collision_object.hpp>

#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>
#include <chrono>
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "tf2/exceptions.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"

//#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2/LinearMath/Quaternion.h>
using std::placeholders::_1;
using std::placeholders::_2;
#include <stdio.h>
void robot_handler(const std::shared_ptr<crust_msgs::srv::RobotCmdSrv::Request> request,
          std::shared_ptr<crust_msgs::srv::RobotCmdSrv::Response>      response)
{
  response->status =  request->cmd;
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Incoming request\na: %ld",
                request->cmd);
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Incoming request\na: %f",
                request->pose[0]);

  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "sending back response: [%ld]", (long int)response->status);

  switch (request->cmd)
  {
  case 1:
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Cmd was\na: %ld",
                request->cmd);
    
    break;
  
  default:
    break;
  }
}
class RobotHandler : public rclcpp::Node
{
  public:
    RobotHandler() : Node("robot_cmd_srv_server")
    {
      service = this->create_service<crust_msgs::srv::RobotCmdSrv>("robot_cmd_srv", std::bind(&RobotHandler::cmdCase, this, _1, _2));
      
      tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
      // Listen to the buffer of transforms
      tf_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf_buffer_);
      // Subscribe to pose published by sensor node (check every second)
      pose_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>
          ("/detected_object", 10,
          std::bind(&RobotHandler::poseCallback, this, std::placeholders::_1));

    }
    
    /*
      cmd:
        1: relative movement quad
        2: relative movement quad with camera offset
        3: packdown robot (Viker)
        4: defalut pos    (Viker)
        5: search 
        6: absolute pose RPY crust_base_link  (Viker)
        7: absolute pose RPY baselink
        8: stop
        9: relative movement rpy
        10: relative movement rpy with camera offset
        11: move relative from base quad
        12: move relative from base rpy 
        13: stop
        14: go to detected object

    */

    void cmdCase(const std::shared_ptr<crust_msgs::srv::RobotCmdSrv::Request> request,
            std::shared_ptr<crust_msgs::srv::RobotCmdSrv::Response>      response)
            {
              switch (request->cmd)
              {
              case 1:
                  response->status = RobotHandler::relativeMovementQuad(request->pose[0],request->pose[1],request->pose[2],request->pose[3],request->pose[4],request->pose[5],request->pose[6]);
                
                break;
              case 2:

                break;
              
              case 3:
                  response->status = RobotHandler::packDownRobot();
                break;
              
              case 4:
                  response->status = RobotHandler::defaultPose();
                break;
              
              case 5:

                break;

              case 6:
                  response->status = RobotHandler::absoluteMovementRPYCrustBase(request->pose[0],request->pose[1],request->pose[2],request->pose[3],request->pose[4],request->pose[5]);

                break;
              
              case 7:

                break;
              
              case 8:

                break;
              
              case 9:
                  response->status = RobotHandler::relativeMovementRPY(request->pose[0],request->pose[1],request->pose[2],request->pose[3],request->pose[4],request->pose[5]);

                break;
              
              case 11:
                  response->status = RobotHandler::baseRelativeMovementQuad(request->pose[0],request->pose[1],request->pose[2],request->pose[3],request->pose[4],request->pose[5],request->pose[6]);
                
                break;
              
              case 12:
                  response->status = RobotHandler::baseRelativeMovementRPY(request->pose[0],request->pose[1],request->pose[2],request->pose[3],request->pose[4],request->pose[5]);
                break;
              
              case 14:
                  response->status = RobotHandler::moveToObject();
                break;
              
              default:
                break;
              }
            }
    
    bool moveToObject(){
      geometry_msgs::msg::TransformStamped target_pose; //= tf_buffer_.lookupTransform("tool_link","crust_base_link");
      
      try {
          //target_pose = tf_buffer_->lookupTransform();
          target_pose = tf_buffer_->lookupTransform(
            "crust_base_link", "tool_link",
            tf2::TimePointZero);
        } catch (const tf2::TransformException & ex) {
          RCLCPP_INFO(
            this->get_logger(), "Could not transform %s to %s: %s",
            "tool_link", "crust_base_link", ex.what());
          return false;
        }
      auto const LOGGER = rclcpp::get_logger("moveToObject");
      RCLCPP_INFO(LOGGER,"x: %f", target_pose.transform.translation.x);
      RCLCPP_INFO(LOGGER,"y: %f", target_pose.transform.translation.y);
      RCLCPP_INFO(LOGGER,"z: %f", target_pose.transform.translation.z);
      RCLCPP_INFO(LOGGER,"x: %f", target_pose.transform.rotation.x);
      RCLCPP_INFO(LOGGER,"y: %f", target_pose.transform.rotation.y);
      RCLCPP_INFO(LOGGER,"z: %f", target_pose.transform.rotation.z);
      RCLCPP_INFO(LOGGER,"x: %f", target_pose.transform.rotation.w);
      return RobotHandler::absoluteMovementQuadCrustBase(pose_out_.pose.position.x,pose_out_.pose.position.y,pose_out_.pose.position.z,pose_out_.pose.orientation.x,pose_out_.pose.orientation.y,pose_out_.pose.orientation.z,pose_out_.pose.orientation.w);
   
      // return RobotHandler::absoluteMovementQuadCrustBase(target_pose.transform.translation.x,target_pose.transform.translation.y,target_pose.transform.translation.z,target_pose.transform.rotation.x,target_pose.transform.rotation.y,target_pose.transform.rotation.z,target_pose.transform.rotation.w);
    }

    bool packDownRobot(){
      double j1 = 0.0;
      double j2 = -1.5;
      double j3 = 0.1;
      double j4 = 1.5;
      return RobotHandler::jointMovement(j1, j2, j3, j4);
    }
    bool defaultPose(){
      double x = 0.39395;
      double y = -0.00778;
      double z = 0.29218;
      double q_x = -1.2562e-05;
      double q_y = 0.22297;
      double q_z = 5.4102e-05;
      double q_w = 0.97482;

      return RobotHandler::absoluteMovementQuadCrustBase(x, y, z, q_x, q_y, q_z, q_w);
    }

    bool jointMovement(double joint1, double joint2, double joint3, double joint4){
              bool status;
              rclcpp::NodeOptions node_options;
              node_options.automatically_declare_parameters_from_overrides(true);
              auto move_group_node = rclcpp::Node::make_shared("robot_server_com_node", node_options);
              rclcpp::executors::MultiThreadedExecutor executor;
              executor.add_node(move_group_node);
              std::thread([&executor]() { executor.spin(); }).detach();
              static const std::string PLANNING_GROUP = "manipulator";
              moveit::planning_interface::MoveGroupInterface move_group(move_group_node, PLANNING_GROUP);
              moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
              const moveit::core::JointModelGroup* joint_model_group = move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP);
              moveit::planning_interface::MoveGroupInterface::Plan my_plan;
              auto const LOGGER = rclcpp::get_logger("absoluteMovementQuadCrustbase");
              std::vector<double> joints = {joint1, joint2, joint3, joint4};
              move_group.setJointValueTarget(joints);
              bool success = static_cast<bool>(move_group.plan(my_plan));
              RCLCPP_INFO(LOGGER, " (baseRelativeMovementQuad) %s", success ? "" : "FAILED");
              if(success == true){
                  //move_group.move();
                  move_group.execute(my_plan);
                  status = true;

                  
              }
              else
              {
                  status = false;
              }
              executor.cancel();
              return status;
    }

    bool absoluteMovementQuadCrustBase(double x, double y, double z, double q_x, double q_y, double q_z, double q_w)
      {
              bool status;
              rclcpp::NodeOptions node_options;
              node_options.automatically_declare_parameters_from_overrides(true);
              auto move_group_node = rclcpp::Node::make_shared("robot_server_com_node", node_options);
              rclcpp::executors::MultiThreadedExecutor executor;
              executor.add_node(move_group_node);
              std::thread([&executor]() { executor.spin(); }).detach();
              static const std::string PLANNING_GROUP = "manipulator";
              moveit::planning_interface::MoveGroupInterface move_group(move_group_node, PLANNING_GROUP);
              moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
              const moveit::core::JointModelGroup* joint_model_group = move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP);
              auto const LOGGER = rclcpp::get_logger("absoluteMovementQuadCrustbase");
             
              move_group.setNumPlanningAttempts(5);

              move_group.setPlanningTime(10.0);

              moveit::planning_interface::MoveGroupInterface::Plan my_plan;

              move_group.setPoseReferenceFrame("crust_base_link");
              geometry_msgs::msg::PoseStamped current_pose;
              
              move_group.setStartStateToCurrentState();
              
              geometry_msgs::msg::Pose pos;
              pos.orientation.x = q_x;
              pos.orientation.y = q_y;
              pos.orientation.z = q_z;
              pos.orientation.w = q_w;
              pos.position.x = x;
              pos.position.y = y;
              pos.position.z = z;
              move_group.setPoseTarget(pos);
              bool success = static_cast<bool>(move_group.plan(my_plan));
              RCLCPP_INFO(LOGGER, " (baseRelativeMovementQuad) %s", success ? "" : "FAILED");
              if(success == true){
                  //move_group.move();
                  move_group.execute(my_plan);
                  status = true;

                  
              }
              else
              {
                  status = false;
              }
              executor.cancel();
              return status;
      }

    bool absoluteMovementRPYCrustBase(double x, double y, double z, double rot_r, double rot_p, double rot_y){
              bool status;
              rclcpp::NodeOptions node_options;
              node_options.automatically_declare_parameters_from_overrides(true);
              auto move_group_node = rclcpp::Node::make_shared("robot_server_com_node", node_options);
              rclcpp::executors::MultiThreadedExecutor executor;
              executor.add_node(move_group_node);
              std::thread([&executor]() { executor.spin(); }).detach();
              static const std::string PLANNING_GROUP = "manipulator";
              moveit::planning_interface::MoveGroupInterface move_group(move_group_node, PLANNING_GROUP);
              moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
              const moveit::core::JointModelGroup* joint_model_group = move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP);
              auto const LOGGER = rclcpp::get_logger("Relative movement quad");
             
              move_group.setNumPlanningAttempts(5);

              move_group.setPlanningTime(10.0);
              //std::vector< double > current_rpy =	move_group.getCurrentRPY();
              moveit::planning_interface::MoveGroupInterface::Plan my_plan;
             
              
              move_group.setPoseReferenceFrame("crust_base_link");
              geometry_msgs::msg::PoseStamped current_pose;
              current_pose = move_group.getCurrentPose();
              
              
              move_group.setStartStateToCurrentState();
              move_group.setPositionTarget(x,y,z);
              move_group.setRPYTarget(rot_r, rot_p, rot_y);
              
              bool success = static_cast<bool>(move_group.plan(my_plan));
              RCLCPP_INFO(LOGGER, " (baseRelativeMovementQuad) %s", success ? "" : "FAILED");
              if(success == true){
                  //move_group.move();
                  move_group.execute(my_plan);
                  status = true;

                  
              }
              else
              {
                  status = false;
              }
              executor.cancel();
              return status;
    }

    void tester(const std::shared_ptr<crust_msgs::srv::RobotCmdSrv::Request> request,
            std::shared_ptr<crust_msgs::srv::RobotCmdSrv::Response>      response)
            { 

              rclcpp::NodeOptions node_options;
              node_options.automatically_declare_parameters_from_overrides(true);
              auto move_group_node = rclcpp::Node::make_shared("move_group_interface_tutorial", node_options);
              // For current state monitor
              rclcpp::executors::MultiThreadedExecutor executor;
              //rclcpp::executors::SingleThreadedExecutor executor;
              executor.add_node(move_group_node);
              std::thread([&executor]() { executor.spin(); }).detach();
              static const std::string PLANNING_GROUP = "manipulator";
              auto const LOGGER = rclcpp::get_logger("hello_moveit");
              moveit::planning_interface::MoveGroupInterface move_group(move_group_node, PLANNING_GROUP);
              moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
              //const moveit::core::JointModelGroup* joint_model_group = move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP);
              
              RCLCPP_INFO(LOGGER, "Planning frame: %s", move_group.getPlanningFrame().c_str());

              // We can also print the name of the end-effector link for this group.
              RCLCPP_INFO(LOGGER, "End effector link: %s", move_group.getEndEffectorLink().c_str());

              // We can get a list of all the groups in the robot:
              RCLCPP_INFO(LOGGER, "Available Planning Groups:");
              std::copy(move_group.getJointModelGroupNames().begin(), move_group.getJointModelGroupNames().end(),std::ostream_iterator<std::string>(std::cout, ", "));
              
              
              response->status =  request->cmd;
              RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Incoming request\na: %ld",
                            request->cmd);
              RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Incoming request\na: %f",
                            request->pose[0]);

              RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "sending back response: [%ld]", (long int)response->status);
              executor.cancel();
            }

    bool relativeMovementQuad(double x, double y, double z, double q_x, double q_y, double q_z, double q_w){
              bool status;
              
              auto move_group_node = rclcpp::Node::make_shared("robot_server_com_node");
              rclcpp::executors::MultiThreadedExecutor executor;
              executor.add_node(move_group_node);
              std::thread([&executor]() { executor.spin(); }).detach();
              static const std::string PLANNING_GROUP = "manipulator";
              moveit::planning_interface::MoveGroupInterface move_group(move_group_node, PLANNING_GROUP);
              moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
              const moveit::core::JointModelGroup* joint_model_group = move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP);
              auto const LOGGER = rclcpp::get_logger("Relative movement quad");
              //const moveit::core::JointModelGroup* joint_model_group = move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP);
    
              move_group.setNumPlanningAttempts(5);

              move_group.setPlanningTime(10.0);

              moveit::planning_interface::MoveGroupInterface::Plan my_plan;

              move_group.setPoseReferenceFrame("tool_link");
              
              move_group.setStartStateToCurrentState();
              
              geometry_msgs::msg::Pose relative_tool;
              relative_tool.orientation.x = q_x;
              relative_tool.orientation.y = q_y;
              relative_tool.orientation.z = q_z;
              relative_tool.orientation.w = q_w;
              relative_tool.position.x = x;
              relative_tool.position.y = y;
              relative_tool.position.z = z;
              move_group.setPoseTarget(relative_tool);
              bool success = static_cast<bool>(move_group.plan(my_plan));
              RCLCPP_INFO(LOGGER, " (relativeMovementQuad) %s", success ? "" : "FAILED");
              if(success == true){
                  //move_group.move();
                  move_group.execute(my_plan);
                  status = true;

                  
              }
              else
              {
                  status = false;
              }
              executor.cancel();
              return status;
      }

    bool relativeMovementRPY(double x, double y, double z, double rot_r, double rot_p, double rot_y){
            bool status;
            
            auto move_group_node = rclcpp::Node::make_shared("robot_server_com_node");
            rclcpp::executors::MultiThreadedExecutor executor;
            executor.add_node(move_group_node);
            std::thread([&executor]() { executor.spin(); }).detach();
            static const std::string PLANNING_GROUP = "manipulator";
            moveit::planning_interface::MoveGroupInterface move_group(move_group_node, PLANNING_GROUP);
            moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
            const moveit::core::JointModelGroup* joint_model_group = move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP);
            auto const LOGGER = rclcpp::get_logger("Relative movement RPY");
            //const moveit::core::JointModelGroup* joint_model_group = move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP);
            move_group.setPoseReferenceFrame("crust_base_link");
            move_group.setNumPlanningAttempts(5);

            move_group.setPlanningTime(10.0);

            moveit::planning_interface::MoveGroupInterface::Plan my_plan;

            geometry_msgs::msg::PoseStamped current_pose;
            current_pose = move_group.getCurrentPose();
            std::vector< double > current_rpy =	move_group.getCurrentRPY();
            tf2::Quaternion q_orig, q_rot, q_new;
            q_orig.setRPY(current_rpy[0],current_rpy[1],current_rpy[2]);
            q_rot.setRPY(rot_r, rot_p, rot_y);
            q_new = q_rot * q_orig;
            q_new.normalize();
            tf2::Vector3 vec_new = tf2::operator*({x,y,x},{current_pose.pose.position.x,current_pose.pose.position.y,current_pose.pose.position.z});
            move_group.setPoseReferenceFrame("tool_link");
            
            move_group.setStartStateToCurrentState();
            geometry_msgs::msg::Pose pos;
            pos.orientation.x = q_new.x();//q_new.x();
            pos.orientation.y = q_new.y();//q_new.y();
            pos.orientation.z = q_new.z();//q_new.z();
            pos.orientation.w = q_new.w();//q_new.w();
            // q_orig.x() = current_pose.pose.orientation.x
            // q_orig.y() = current_pose.pose.orientation.y
            // q_orig.z() = current_pose.pose.orientation.z
            // q_orig.w() = current_pose.pose.orientation.w

            pos.position.x = vec_new[0];
            pos.position.y = vec_new[1];
            pos.position.z = vec_new[2];
            RCLCPP_INFO(LOGGER,"x: %f", pos.position.x);
            RCLCPP_INFO(LOGGER,"y: %f", pos.position.y);
            RCLCPP_INFO(LOGGER,"z: %f", pos.position.z);
            RCLCPP_INFO(LOGGER,"rot x: %f", pos.orientation.x);
            RCLCPP_INFO(LOGGER,"rot y: %f", pos.orientation.y);
            RCLCPP_INFO(LOGGER,"rot z: %f", pos.orientation.z);
            RCLCPP_INFO(LOGGER,"rot w: %f", pos.orientation.w);
            //move_group.setPoseTarget(pos);
          
            move_group.setRPYTarget(rot_r,rot_p,rot_y);
            move_group.setPositionTarget(x,y,z);
            bool success = static_cast<bool>(move_group.plan(my_plan));
            RCLCPP_INFO(LOGGER, " (relativeMovementRPY) %s", success ? "" : "FAILED");
            if(success == true){
                move_group.move();
                //move_group.execute(my_plan);
                status = true;

                
            }
            else
            {
                status = false;
            }
            executor.cancel();
            return status;
  }

    bool baseRelativeMovementQuad(double x, double y, double z, double q_x, double q_y, double q_z, double q_w)
    {
              bool status;
              rclcpp::NodeOptions node_options;
              node_options.automatically_declare_parameters_from_overrides(true);
              auto move_group_node = rclcpp::Node::make_shared("robot_server_com_node", node_options);
              rclcpp::executors::MultiThreadedExecutor executor;
              executor.add_node(move_group_node);
              std::thread([&executor]() { executor.spin(); }).detach();
              static const std::string PLANNING_GROUP = "manipulator";
              moveit::planning_interface::MoveGroupInterface move_group(move_group_node, PLANNING_GROUP);
              moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
              const moveit::core::JointModelGroup* joint_model_group = move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP);
              auto const LOGGER = rclcpp::get_logger("Base Relative movement quad");
             
              move_group.setNumPlanningAttempts(5);

              move_group.setPlanningTime(10.0);

              moveit::planning_interface::MoveGroupInterface::Plan my_plan;

              move_group.setPoseReferenceFrame("crust_base_link");
              geometry_msgs::msg::PoseStamped current_pose;
              current_pose = move_group.getCurrentPose();
              
              move_group.setStartStateToCurrentState();
              
              geometry_msgs::msg::Pose relative_tool;
              relative_tool.orientation.x = current_pose.pose.orientation.x+q_x;
              relative_tool.orientation.y = current_pose.pose.orientation.y+q_y;
              relative_tool.orientation.z = current_pose.pose.orientation.z+q_z;
              relative_tool.orientation.w = current_pose.pose.orientation.w+q_w;
              relative_tool.position.x = current_pose.pose.position.x+x;
              relative_tool.position.y = current_pose.pose.position.y+y;
              relative_tool.position.z = current_pose.pose.position.z+z;
              move_group.setPoseTarget(relative_tool);
              bool success = static_cast<bool>(move_group.plan(my_plan));
              RCLCPP_INFO(LOGGER, " (baseRelativeMovementQuad) %s", success ? "" : "FAILED");
              if(success == true){
                  //move_group.move();
                  move_group.execute(my_plan);
                  status = true;

                  
              }
              else
              {
                  status = false;
              }
              executor.cancel();
              return status;
    }

    bool baseRelativeMovementRPY(double x, double y, double z, double rot_r, double rot_p, double rot_y)
    {
              bool status;
              rclcpp::NodeOptions node_options;
              node_options.automatically_declare_parameters_from_overrides(true);
              auto move_group_node = rclcpp::Node::make_shared("robot_server_com_node", node_options);
              rclcpp::executors::MultiThreadedExecutor executor;
              executor.add_node(move_group_node);
              std::thread([&executor]() { executor.spin(); }).detach();
              static const std::string PLANNING_GROUP = "manipulator";
              moveit::planning_interface::MoveGroupInterface move_group(move_group_node, PLANNING_GROUP);
              moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
              const moveit::core::JointModelGroup* joint_model_group = move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP);
              auto const LOGGER = rclcpp::get_logger("BaseRelative movement RPY");
             
              move_group.setNumPlanningAttempts(10);
              move_group.setPoseReferenceFrame("crust_base_link");
              move_group.setPlanningTime(10.0);
              move_group.setGoalOrientationTolerance(0.005);
              move_group.setGoalPositionTolerance(0.005);

              //move_group.setMaxVelocityScalingFactor(0.05);
              //move_group.setMaxAccelerationScalingFactor(0.05);
              std::vector< double > current_rpy =	move_group.getCurrentRPY();
              moveit::planning_interface::MoveGroupInterface::Plan my_plan;
              tf2::Quaternion q_orig, q_rot, q_new;
              q_orig.setRPY(current_rpy[0],current_rpy[1],current_rpy[2]);
              q_rot.setRPY(rot_r, rot_p, rot_y);
              q_new = q_rot * q_orig;
              q_new.normalize();


              
              
              geometry_msgs::msg::PoseStamped current_pose;
              current_pose = move_group.getCurrentPose();
              
              
              move_group.setStartStateToCurrentState();
              
              geometry_msgs::msg::Pose pos;
              pos.orientation.x = q_new.x();//q_new.x();
              pos.orientation.y = q_new.y();//q_new.y();
              pos.orientation.z = q_new.z();//q_new.z();
              pos.orientation.w = q_new.w();//q_new.w();
              // q_orig.x() = current_pose.pose.orientation.x
              // q_orig.y() = current_pose.pose.orientation.y
              // q_orig.z() = current_pose.pose.orientation.z
              // q_orig.w() = current_pose.pose.orientation.w

              pos.position.x = current_pose.pose.position.x+x;
              pos.position.y = current_pose.pose.position.y+y;
              pos.position.z = current_pose.pose.position.z+z;
              move_group.setPoseTarget(pos);
              bool success = static_cast<bool>(move_group.plan(my_plan));
              RCLCPP_INFO(LOGGER, " (baseRelativeMovementRPY) %s", success ? "" : "FAILED");
              if(success == true){
                  //move_group.move();
                  move_group.execute(my_plan);
                  status = true;

                  
              }
              else
              {
                  status = false;
              }
              executor.cancel();
              return status;
    }
  private:
    rclcpp::Service<crust_msgs::srv::RobotCmdSrv>::SharedPtr service;

        // Subscription to pose published by sensor node
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;
    // Listener for the broadcast transform message
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    // Buffer that stores several seconds of transforms for easy lookup by the listener
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    // Pose in source frame (`sensor_link`)
    geometry_msgs::msg::PoseStamped pose_in_;
    // Pose in target frame (`arm_end_link`)
    geometry_msgs::msg::PoseStamped pose_out_;

    void poseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg){
        try {
            pose_in_ = *msg;
            // Transforms the pose between the source frame and target frame
            tf_buffer_->transform<geometry_msgs::msg::PoseStamped>(pose_in_, pose_out_, "crust_base_link", 
                tf2::Duration(std::chrono::seconds(1)));
            // Log coordinates of pose in target frame
            RCLCPP_INFO(get_logger(),"Object pose in 'tool_link' is:\n x,y,z = %.1f,%.1f,%.1f",
                pose_out_.pose.position.x,
                pose_out_.pose.position.y,
                pose_out_.pose.position.z);
        } catch (const tf2::TransformException & ex){
            RCLCPP_WARN(get_logger(),"Could not find object position in 'arm_end_link' frame.");
            return;
        }
    }
    
};




int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);

  // rclcpp::NodeOptions node_options;
  // node_options.automatically_declare_parameters_from_overrides(true);
  // auto move_group_node = rclcpp::Node::make_shared("move_group_interface_tutorial", node_options);
  // // For current state monitor
  // rclcpp::executors::SingleThreadedExecutor executor;
  // //rclcpp::executors::MultiThreadedExecutor executor;
  // executor.add_node(move_group_node);
  // std::thread([&executor]() { executor.spin(); }).detach();
  // static const std::string PLANNING_GROUP = "manipulator";
  // auto const LOGGER = rclcpp::get_logger("hello_moveit");
  // moveit::planning_interface::MoveGroupInterface move_group(move_group_node, PLANNING_GROUP);
  // moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
  // const moveit::core::JointModelGroup* joint_model_group = move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP);
  
  // RCLCPP_INFO(LOGGER, "Planning frame: %s", move_group.getPlanningFrame().c_str());

  // // We can also print the name of the end-effector link for this group.
  // RCLCPP_INFO(LOGGER, "End effector link: %s", move_group.getEndEffectorLink().c_str());

  // // We can get a list of all the groups in the robot:
  // RCLCPP_INFO(LOGGER, "Available Planning Groups:");
  // std::copy(move_group.getJointModelGroupNames().begin(), move_group.getJointModelGroupNames().end(),std::ostream_iterator<std::string>(std::cout, ", "));
  // sleep(5.0);
  // //executor.cancel();
  
  
  
  
  
  
  // rclcpp::NodeOptions node_options;
  // node_options.automatically_declare_parameters_from_overrides(true);

  // std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("robot_cmd_srv_server", node_options);
  // //auto move_group_node = rclcpp::Node::make_shared("hello2", node_options);
  // rclcpp::executors::SingleThreadedExecutor executor;
  
  
  // executor.add_node(node);
  // std::thread([&executor]() { executor.spin(); }).detach();
  // static const std::string PLANNING_GROUP = "manipulator";
  // moveit::planning_interface::MoveGroupInterface move_group(node, PLANNING_GROUP);
  // moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
  // const moveit::core::JointModelGroup* joint_model_group = move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP);
  // auto const LOGGER = rclcpp::get_logger("hello_moveit");
  // RCLCPP_INFO(LOGGER, "Planning frame: %s", move_group.getPlanningFrame().c_str());

  // // We can also print the name of the end-effector link for this group.
  // RCLCPP_INFO(LOGGER, "End effector link: %s", move_group.getEndEffectorLink().c_str());

  // // We can get a list of all the groups in the robot:
  // RCLCPP_INFO(LOGGER, "Available Planning Groups:");
  // std::copy(move_group.getJointModelGroupNames().begin(), move_group.getJointModelGroupNames().end(),std::ostream_iterator<std::string>(std::cout, ", "));





  // rclcpp::Service<crust_msgs::srv::RobotCmdSrv>::SharedPtr service =
  //   node->create_service<crust_msgs::srv::RobotCmdSrv>("robot_cmd_srv", &robot_handler);

  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Server Ready.");
  rclcpp::spin(std::make_shared<RobotHandler>());
  
  rclcpp::shutdown();
}