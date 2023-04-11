#define BOOST_BIND_NO_PLACEHOLDERS

#include "rclcpp/rclcpp.hpp"
#include <rclcpp/qos.hpp>

//#include "crust_msgs/srv/RobotCmdSrv.hpp"
#include <crust_msgs/srv/robot_cmd_srv.hpp>
#include <crust_msgs/msg/robot_cmd_msg.hpp>
//#include <crust_msgs/msg/robot_cmd_msg.h>
//#include <crust_msgs/srv/robot_cmd_srv.h>
#include <memory>
#include <vector>
#include <cmath>

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
#include <std_msgs/msg/int8.hpp>

//#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2/LinearMath/Quaternion.h>
using std::placeholders::_1;
using std::placeholders::_2;
using std::placeholders::_3;
#include <stdio.h>

const std::string MOVEGROUPGRIPPER = "gripper";
const std::string MOVEGROUPMANIPULATOR = "manipulator";
//std::shared_ptr<rclcpp::Node> node = nullptr;
//rclcpp::executors::MultiThreadedExecutor executor;


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

  private:

    moveit::planning_interface::MoveGroupInterface move_group;
    moveit::planning_interface::MoveGroupInterface move_group_gripper;
    const moveit::core::JointModelGroup* joint_model_group_gripper;
    const moveit::core::JointModelGroup* joint_model_group;
    moveit::planning_interface::MoveGroupInterface::Plan my_plan;

  public:
    RobotHandler() : Node("robot_cmd_srv_server"), move_group(std::shared_ptr<rclcpp::Node>(std::move(this)), MOVEGROUPMANIPULATOR), move_group_gripper(std::shared_ptr<rclcpp::Node>(std::move(this)), MOVEGROUPGRIPPER)
    {
      service = this->create_service<crust_msgs::srv::RobotCmdSrv>("robot_cmd_srv", std::bind(&RobotHandler::cmdCase, this, _1, _2));
      
      tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
      // Listen to the buffer of transforms
      tf_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf_buffer_);
      // Subscribe to pose published by sensor node (check every second)
      pose_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>
          ("/detected_object", rclcpp::QoS(1),
          std::bind(&RobotHandler::poseCallback, this, std::placeholders::_1));
      //cmd_sub = create_subscription<<crust_msgs::srv::RobotCmdSrv>("/cmd_msg",10,std::bind(&RobotHandler::cmdCallback, this, std::placeholders::_3));
      //status_pub = 
      cmd_sub_ = this->create_subscription<crust_msgs::msg::RobotCmdMsg>("/robot_cmd_msg",rclcpp::QoS(1),std::bind(&RobotHandler::cmdCaseMsg, this, std::placeholders::_1));
      cmd_pub_ = this->create_publisher<std_msgs::msg::Int8>("/robot_cmd_status",10);
      RCLCPP_INFO(this->get_logger(), "Initialization successful.");

      

    }
    
    /*
      cmd:
        1: relative movement quad
        2: relative movement quad with camera offset
        3: packdown robot (Viker)
        4: defalut pos    (Viker)
        5: search 
        6: absolute pose RPY crust_base_link  (Viker)
        7: absoluteMovementQuadCrustBase crust_baselink
        8: joint movement
        9: relative movement rpy
        10: relative movement rpy with camera offset
        11: move relative from base quad
        12: move relative from base rpy 
        13: stop
        14: go to detected object
        15: go to arcro
        16: go to golfball
        17: Cartesian path w. tolerance
        18: gripper On
        19: gripper off
        20: gripper set joint value
        21: relative joint

    */
    void cmdCaseMsg(const crust_msgs::msg::RobotCmdMsg::SharedPtr msg){
        robot_msg = *msg;
         switch (robot_msg.cmd)
              {
              case 1:
                  status_msg.data = RobotHandler::relativeMovementQuad(robot_msg.pose[0],robot_msg.pose[1],robot_msg.pose[2],robot_msg.pose[3],robot_msg.pose[4],robot_msg.pose[5],robot_msg.pose[6]);
                
                break;
              case 2:

                break;
              
              case 3:
                  status_msg.data = RobotHandler::packDownRobot();
                break;
              
              case 4:
                  status_msg.data = RobotHandler::defaultPose();
                break;
              
              case 5:

                break;

              case 6:
                  status_msg.data = RobotHandler::absoluteMovementRPYCrustBase(robot_msg.pose[0],robot_msg.pose[1],robot_msg.pose[2],robot_msg.pose[3],robot_msg.pose[4],robot_msg.pose[5]);

                break;
              
              case 7:
                  status_msg.data = RobotHandler::absoluteMovementQuadCrustBase(robot_msg.pose[0],robot_msg.pose[1],robot_msg.pose[2],robot_msg.pose[3],robot_msg.pose[4],robot_msg.pose[5],robot_msg.pose[6]);
                
                break;
              
              case 8:
                  status_msg.data = RobotHandler::jointMovement(robot_msg.pose[0],robot_msg.pose[1],robot_msg.pose[2],robot_msg.pose[3]);
                break;
              
              case 9:
                  status_msg.data = RobotHandler::relativeMovementRPY(robot_msg.pose[0],robot_msg.pose[1],robot_msg.pose[2],robot_msg.pose[3],robot_msg.pose[4],robot_msg.pose[5]);

                break;
              
              case 11:
                  status_msg.data = RobotHandler::baseRelativeMovementQuad(robot_msg.pose[0],robot_msg.pose[1],robot_msg.pose[2],robot_msg.pose[3],robot_msg.pose[4],robot_msg.pose[5],robot_msg.pose[6]);
                
                break;
              
              case 12:
                  status_msg.data = RobotHandler::baseRelativeMovementRPY(robot_msg.pose[0],robot_msg.pose[1],robot_msg.pose[2],robot_msg.pose[3],robot_msg.pose[4],robot_msg.pose[5]);
                break;
              
              case 13:
                  status_msg.data = RobotHandler::stop();

                break;
              
              case 14:
                  status_msg.data = RobotHandler::moveToObject();
                break;
              
              case 15:
                  status_msg.data = RobotHandler::moveToAruco(robot_msg.pose[0],robot_msg.pose[1],robot_msg.pose[2]);
                break;
              
              case 16:
                  status_msg.data = RobotHandler::moveToBall(robot_msg.pose[0],robot_msg.pose[1],robot_msg.pose[2]);
                break;
              
              case 17:
                status_msg.data = RobotHandler::CartesianPath(robot_msg.pose[0],robot_msg.pose[1],robot_msg.pose[2],robot_msg.pose[3],robot_msg.pose[4],robot_msg.pose[5],robot_msg.pose[6],robot_msg.pose[7]);
                break;
              
              case 18:
                status_msg.data = RobotHandler::gripperOn();
                break;

              case 19:
                status_msg.data = RobotHandler::gripperOff();
                break;

              case 20:
                status_msg.data = RobotHandler::gripperSetValue(robot_msg.pose[0],robot_msg.pose[1]);
                break;
              
              case 21:
                status_msg.data = RobotHandler::RealativeJointMovement(robot_msg.pose[0],robot_msg.pose[1],robot_msg.pose[2],robot_msg.pose[3]);
                break;
              
              case 22:
                status_msg.data = RobotHandler::moveToBallTrolly(robot_msg.pose[0],robot_msg.pose[1],robot_msg.pose[2]);
                break;

              default:
                break;
              }
              cmd_pub_ ->publish(status_msg);

    }
    bool gripperSetValue(double j1, double j2){
        bool status;
        //rclcpp::NodeOptions node_options;
        //node_options.automatically_declare_parameters_from_overrides(true);
        //auto move_group_node = rclcpp::Node::make_shared("robot_server_com_node", node_options);
        //rclcpp::executors::MultiThreadedExecutor executor;
        //executor.add_node(move_group_node);
        //std::thread([&executor]() { executor.spin(); }).detach();
        //static const std::string PLANNING_GROUP = "gripper";
        //moveit::planning_interface::MoveGroupInterface move_group(move_group_node, PLANNING_GROUP);
        //moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
        //const moveit::core::JointModelGroup* joint_model_group = move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP);
        //joint_model_group_gripper = this->move_group_gripper.getCurrentState()->getJointModelGroup(MOVEGROUPGRIPPER);
        auto const LOGGER = rclcpp::get_logger("gripper On");
        //moveit::planning_interface::MoveGroupInterface::Plan my_plan;
        std::vector<double> joints = {j1, j2};
        move_group_gripper.setJointValueTarget(joints);
        bool success = static_cast<bool>(this->move_group_gripper.plan(my_plan));
        RCLCPP_INFO(LOGGER, " (gripper On) %s", success ? "" : "FAILED");
        if(success == true){
            //move_group.move();
            this->move_group.execute(my_plan);
            status = true;

            
        }
        else
        {
            status = false;
        }
        //executor.cancel();
        return status;

    }

    bool gripperOn(){
        bool status;
        // rclcpp::NodeOptions node_options;
        // node_options.automatically_declare_parameters_from_overrides(true);
        // auto move_group_node = rclcpp::Node::make_shared("robot_server_com_node", node_options);
        // rclcpp::executors::MultiThreadedExecutor executor;
        // executor.add_node(move_group_node);
        // std::thread([&executor]() { executor.spin(); }).detach();
        // static const std::string PLANNING_GROUP = "gripper";
        // moveit::planning_interface::MoveGroupInterface move_group(move_group_node, PLANNING_GROUP);
        // moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
        
        //const moveit::core::JointModelGroup* joint_model_group = move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP);
        
        //executor2.add_node(node);
        //std::thread([&executor]() { executor.spin(); }).detach();
        
        
        //joint_model_group_gripper = this->move_group_gripper.getCurrentState()->getJointModelGroup(MOVEGROUPGRIPPER);
        auto const LOGGER = rclcpp::get_logger("gripper On");
        //moveit::planning_interface::MoveGroupInterface::Plan my_plan;
        std::vector<double> joints = {0.13962634, 0.13962634};
        std::vector<std::string> joint_names = {"finger1_joint", "finger2_joint"};
        this->move_group_gripper.setJointValueTarget(joint_names,joints);
        bool success = static_cast<bool>(this->move_group_gripper.plan(my_plan));
        RCLCPP_INFO(LOGGER, " (gripper On) %s", success ? "" : "FAILED");
        if(success == true){
            //move_group.move();
            this->move_group_gripper.execute(my_plan);
            status = true;

            
        }
        else
        {
            status = false;
        }
        //executor.cancel();
        return status;

    }

    bool gripperOff(){
        bool status;
        // rclcpp::NodeOptions node_options;
        // node_options.automatically_declare_parameters_from_overrides(true);
        // auto move_group_node = rclcpp::Node::make_shared("robot_server_com_node", node_options);
        // rclcpp::executors::MultiThreadedExecutor executor;
        // executor.add_node(move_group_node);
        // std::thread([&executor]() { executor.spin(); }).detach();
        // static const std::string PLANNING_GROUP = "gripper";
        // moveit::planning_interface::MoveGroupInterface move_group(move_group_node, PLANNING_GROUP);
        // moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
        // const moveit::core::JointModelGroup* joint_model_group = move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP);
        auto const LOGGER = rclcpp::get_logger("gripper Off");
        // moveit::planning_interface::MoveGroupInterface::Plan my_plan;
        std::vector<double> joints = {-0.40, -0.40};
        std::vector<std::string> joint_names = {"finger1_joint", "finger2_joint"};
        this->move_group_gripper.setJointValueTarget(joint_names,joints);
        //move_group.setJointValueTarget(joints);
        bool success = static_cast<bool>(this->move_group_gripper.plan(my_plan));
        RCLCPP_INFO(LOGGER, " (gripper On) %s", success ? "" : "FAILED");
        if(success == true){
            //move_group.move();
            this->move_group_gripper.execute(my_plan);
            status = true;

            
        }
        else
        {
            status = false;
        }
        //executor.cancel();
        return status;
    }

    bool CartesianPath(double x, double y, double z, double q_x, double q_y, double q_z, double q_w, double orientation_tolerance){
        std::vector<geometry_msgs::msg::Pose> waypoints;
        bool status;
        // rclcpp::NodeOptions node_options;
        // node_options.automatically_declare_parameters_from_overrides(true);
        // auto move_group_node = rclcpp::Node::make_shared("robot_server_com_node", node_options);
        // rclcpp::executors::MultiThreadedExecutor executor;
        // executor.add_node(move_group_node);
        // std::thread([&executor]() { executor.spin(); }).detach();
        // static const std::string PLANNING_GROUP = "manipulator";
        // moveit::planning_interface::MoveGroupInterface move_group(move_group_node, PLANNING_GROUP);
        // moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
        // const moveit::core::JointModelGroup* joint_model_group = move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP);
        auto const LOGGER = rclcpp::get_logger("absoluteMovementQuadCrustbase");
        
        this->move_group.setNumPlanningAttempts(5);

        this->move_group.setPlanningTime(10.0);
        //move_group.setGoalOrientationTolerance(orientation_tolerance);

        //moveit::planning_interface::MoveGroupInterface::Plan my_plan;

        this->move_group.setPoseReferenceFrame("crust_base_link");
        //geometry_msgs::msg::PoseStamped current_pose;
        
        //this->move_group.setStartStateToCurrentState();

        
        geometry_msgs::msg::Pose pos;
        pos.orientation.x = q_x;
        pos.orientation.y = q_y;
        pos.orientation.z = q_z;
        pos.orientation.w = q_w;
        pos.position.x = x;
        pos.position.y = y;
        pos.position.z = z;
        pos.position.z += 0.1;
        //waypoints.push_back(pos);
        pos.position.z -= 0.05;
        waypoints.push_back(pos);
        pos.position.z = z;
        waypoints.push_back(pos);
        moveit_msgs::msg::RobotTrajectory trajectory;
        const double jump_threshold = 0.0;
        const double eef_step = 0.01;
        double fraction = this->move_group.computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);
        this->move_group.execute(trajectory);

        if(fraction < 0.0){
           
            status = false;

            
        }
        else
        {
            status = true;
        }
        //executor.cancel();
        return status;


    }

    bool stop(){
      bool status;
      // rclcpp::NodeOptions node_options;
      // node_options.automatically_declare_parameters_from_overrides(true);
      // auto move_group_node = rclcpp::Node::make_shared("robot_server_com_node", node_options);
      // rclcpp::executors::MultiThreadedExecutor executor;
      // executor.add_node(move_group_node);
      // std::thread([&executor]() { executor.spin(); }).detach();
      // static const std::string PLANNING_GROUP = "manipulator";
      // moveit::planning_interface::MoveGroupInterface move_group(move_group_node, PLANNING_GROUP);
      // moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
      // const moveit::core::JointModelGroup* joint_model_group = move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP);
      auto const LOGGER = rclcpp::get_logger("stop");
      this->move_group.stop();
      status = true;
      return status;

    }
    bool moveToBallTrolly(double x, double y, double z){
      geometry_msgs::msg::TransformStamped target_pose; //= tf_buffer_.lookupTransform("tool_link","crust_base_link");
      
      try {
          //target_pose = tf_buffer_->lookupTransform();
          target_pose = tf_buffer_->lookupTransform(
            "crust_base_link", "ball",
            tf2::TimePointZero);
          auto const LOGGER = rclcpp::get_logger("moveToObject");
          RCLCPP_INFO(LOGGER,"x: %f", target_pose.transform.translation.x);
          RCLCPP_INFO(LOGGER,"y: %f", target_pose.transform.translation.y);
          RCLCPP_INFO(LOGGER,"z: %f", target_pose.transform.translation.z);
          RCLCPP_INFO(LOGGER,"x: %f", target_pose.transform.rotation.x);
          RCLCPP_INFO(LOGGER,"y: %f", target_pose.transform.rotation.y);
          RCLCPP_INFO(LOGGER,"z: %f", target_pose.transform.rotation.z);
          RCLCPP_INFO(LOGGER,"x: %f", target_pose.transform.rotation.w);
          tf2::Quaternion quat_tf;
          
          tf2::fromMsg(target_pose.transform.rotation, quat_tf);
          double rot_r{}, rot_p{}, rot_y{};
          tf2::Matrix3x3 m(quat_tf);
          m.getRPY(rot_r, rot_p, rot_y);
          
          quat_tf.setRPY(0.0,1.57,atan2(target_pose.transform.translation.y, target_pose.transform.translation.x));
          RCLCPP_INFO(LOGGER,"R: %f", rot_r);
          RCLCPP_INFO(LOGGER,"P: %f", rot_p);
          RCLCPP_INFO(LOGGER,"Y: %f", rot_y);
          target_pose.transform.rotation = tf2::toMsg(quat_tf);
          bool i = RobotHandler::absoluteMovementQuadCrustBaseWOrientationTolerance(target_pose.transform.translation.x + x,target_pose.transform.translation.y + y,target_pose.transform.translation.z + z+0.1,target_pose.transform.rotation.x,target_pose.transform.rotation.y,target_pose.transform.rotation.z,target_pose.transform.rotation.w, 0.5);
          if(i == 1){

            i =  RobotHandler::CartesianPath(target_pose.transform.translation.x + x,target_pose.transform.translation.y + y,target_pose.transform.translation.z + z-0.02,target_pose.transform.rotation.x,target_pose.transform.rotation.y,target_pose.transform.rotation.z,target_pose.transform.rotation.w, 0.05);
          }
          else{
            return false;
          }
          if(i == 1){
            i = RobotHandler::gripperOn();
          }
          else
          {
            return false;
          }
          if(i == 1)
          {
            return RobotHandler::absoluteMovementQuadCrustBaseWOrientationTolerance(target_pose.transform.translation.x + x,target_pose.transform.translation.y + y,target_pose.transform.translation.z + z+0.1,target_pose.transform.rotation.x,target_pose.transform.rotation.y,target_pose.transform.rotation.z,target_pose.transform.rotation.w, 0.5);
          }
          else
          {
            return false;
          }

        } catch (const tf2::TransformException & ex) {
          RCLCPP_INFO(
            this->get_logger(), "Could not transform %s to %s: %s",
            "ball", "crust_base_link", ex.what());
          return false;
        }
    }


    bool moveToBall(double x, double y, double z){
      geometry_msgs::msg::TransformStamped target_pose; //= tf_buffer_.lookupTransform("tool_link","crust_base_link");
      
      try {
          //target_pose = tf_buffer_->lookupTransform();
          target_pose = tf_buffer_->lookupTransform(
            "crust_base_link", "ball",
            tf2::TimePointZero);
          auto const LOGGER = rclcpp::get_logger("moveToObjectball");
          RCLCPP_INFO(LOGGER,"x: %f", target_pose.transform.translation.x);
          RCLCPP_INFO(LOGGER,"y: %f", target_pose.transform.translation.y);
          RCLCPP_INFO(LOGGER,"z: %f", target_pose.transform.translation.z);
          RCLCPP_INFO(LOGGER,"x: %f", target_pose.transform.rotation.x);
          RCLCPP_INFO(LOGGER,"y: %f", target_pose.transform.rotation.y);
          RCLCPP_INFO(LOGGER,"z: %f", target_pose.transform.rotation.z);
          RCLCPP_INFO(LOGGER,"x: %f", target_pose.transform.rotation.w);
          tf2::Quaternion quat_tf;
          
          tf2::fromMsg(target_pose.transform.rotation, quat_tf);
          double rot_r{}, rot_p{}, rot_y{};
          tf2::Matrix3x3 m(quat_tf);
          m.getRPY(rot_r, rot_p, rot_y);
          
          quat_tf.setRPY(0.0,1.57,atan2(target_pose.transform.translation.y, target_pose.transform.translation.x));
          //quat_tf.setRPY(rot_r, rot_p, rot_y);
          quat_tf.normalize();
          RCLCPP_INFO(LOGGER,"R: %f", rot_r);
          RCLCPP_INFO(LOGGER,"P: %f", rot_p);
          RCLCPP_INFO(LOGGER,"Y: %f", rot_y);
          target_pose.transform.rotation = tf2::toMsg(quat_tf);
          bool i = RobotHandler::absoluteMovementRPYCrustBase(target_pose.transform.translation.x + x,target_pose.transform.translation.y + y,target_pose.transform.translation.z + z+0.1,0.0,0.157,atan2(target_pose.transform.translation.y, target_pose.transform.translation.x));
          //bool i = RobotHandler::absoluteMovementQuadCrustBaseWOrientationTolerance(target_pose.transform.translation.x + x,target_pose.transform.translation.y + y,target_pose.transform.translation.z + z+0.1,target_pose.transform.rotation.x,target_pose.transform.rotation.y,target_pose.transform.rotation.z,target_pose.transform.rotation.w, 0.7);
          
          if(i == 1){

            i =  RobotHandler::CartesianPath(target_pose.transform.translation.x + x,target_pose.transform.translation.y + y,target_pose.transform.translation.z + z-0.03,target_pose.transform.rotation.x,target_pose.transform.rotation.y,target_pose.transform.rotation.z,target_pose.transform.rotation.w, 0.05);
          }
          else{
            return false;
          }
          if(i == 1){
            i = RobotHandler::gripperOn();
          }
          else
          {
            return false;
          }
          if(i == 1)
          {
            return RobotHandler::absoluteMovementQuadCrustBaseWOrientationTolerance(target_pose.transform.translation.x + x,target_pose.transform.translation.y + y,target_pose.transform.translation.z + z+0.1,target_pose.transform.rotation.x,target_pose.transform.rotation.y,target_pose.transform.rotation.z,target_pose.transform.rotation.w, 0.5);
          }
          else
          {
            return false;
          }

        } catch (const tf2::TransformException & ex) {
          RCLCPP_INFO(
            this->get_logger(), "Could not transform %s to %s: %s",
            "ball", "crust_base_link", ex.what());
          return false;
        }
    }

    bool moveToAruco(double x, double y, double z){
      geometry_msgs::msg::TransformStamped target_pose; //= tf_buffer_.lookupTransform("tool_link","crust_base_link");
      
      try {
          //target_pose = tf_buffer_->lookupTransform();
          target_pose = tf_buffer_->lookupTransform(
            "crust_base_link", "aruco",
            tf2::TimePointZero);
          auto const LOGGER = rclcpp::get_logger("moveToObjectaruco");
          RCLCPP_INFO(LOGGER,"x: %f", target_pose.transform.translation.x);
          RCLCPP_INFO(LOGGER,"y: %f", target_pose.transform.translation.y);
          RCLCPP_INFO(LOGGER,"z: %f", target_pose.transform.translation.z);
          RCLCPP_INFO(LOGGER,"x: %f", target_pose.transform.rotation.x);
          RCLCPP_INFO(LOGGER,"y: %f", target_pose.transform.rotation.y);
          RCLCPP_INFO(LOGGER,"z: %f", target_pose.transform.rotation.z);
          RCLCPP_INFO(LOGGER,"x: %f", target_pose.transform.rotation.w);
          
          tf2::Quaternion quat_tf;
          
          tf2::fromMsg(target_pose.transform.rotation, quat_tf);
          double rot_r{}, rot_p{}, rot_y{};
          tf2::Matrix3x3 m(quat_tf);
          m.getRPY(rot_r, rot_p, rot_y);
          quat_tf.setRPY(0.0,1.57,atan2(target_pose.transform.translation.y, target_pose.transform.translation.x));
          RCLCPP_INFO(LOGGER,"R: %f", rot_r);
          RCLCPP_INFO(LOGGER,"P: %f", rot_p);
          RCLCPP_INFO(LOGGER,"Y: %f", rot_y);
          target_pose.transform.rotation = tf2::toMsg(quat_tf);

          //return RobotHandler::absoluteMovementQuadCrustBaseWOrientationTolerance(target_pose.transform.translation.x + x,target_pose.transform.translation.y + y,target_pose.transform.translation.z + z,target_pose.transform.rotation.x,target_pose.transform.rotation.y,target_pose.transform.rotation.z,target_pose.transform.rotation.w, 0.4);
          //return RobotHandler::CartesianPath(target_pose.transform.translation.x + x,target_pose.transform.translation.y + y,target_pose.transform.translation.z + z,target_pose.transform.rotation.x,target_pose.transform.rotation.y,target_pose.transform.rotation.z,target_pose.transform.rotation.w, 0.5);
          bool i = RobotHandler::absoluteMovementQuadCrustBaseWOrientationTolerance(target_pose.transform.translation.x + x -0.01,target_pose.transform.translation.y + y,target_pose.transform.translation.z + z+0.1,target_pose.transform.rotation.x,target_pose.transform.rotation.y,target_pose.transform.rotation.z,target_pose.transform.rotation.w, 0.5);
          if(i == 1){

            i = RobotHandler::CartesianPath(target_pose.transform.translation.x + x -0.01,target_pose.transform.translation.y + y,target_pose.transform.translation.z + z -0.03,target_pose.transform.rotation.x,target_pose.transform.rotation.y,target_pose.transform.rotation.z,target_pose.transform.rotation.w, 0.05);
          }
          else{
            return false;
          }
          if(i == 1){
            i = RobotHandler::gripperOn();
          }
          else
          {
            return false;
          }
          if(i == 1)
          {
            return RobotHandler::absoluteMovementQuadCrustBaseWOrientationTolerance(target_pose.transform.translation.x + x -0.01,target_pose.transform.translation.y + y,target_pose.transform.translation.z + z+0.1,target_pose.transform.rotation.x,target_pose.transform.rotation.y,target_pose.transform.rotation.z,target_pose.transform.rotation.w, 0.5);
          }
          else
          {
            return false;
          }
        } catch (const tf2::TransformException & ex) {
          RCLCPP_INFO(
            this->get_logger(), "Could not transform %s to %s: %s",
            "aruco", "crust_base_link", ex.what());
          return false;
        }
    }

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
                  response->status = RobotHandler::absoluteMovementQuadCrustBase(request->pose[0],request->pose[1],request->pose[2],request->pose[3],request->pose[4],request->pose[5],request->pose[6]);
                break;
              
              case 8:
                  response->status = RobotHandler::jointMovement(request->pose[0],request->pose[1],request->pose[2],request->pose[3]);
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
      double j2 = 1.22173048;
      double j3 = 1.85004901;
      double j4 = -1.74532925;
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
              // rclcpp::NodeOptions node_options;
              // node_options.automatically_declare_parameters_from_overrides(true);
              // auto move_group_node = rclcpp::Node::make_shared("robot_server_com_node", node_options);
              // rclcpp::executors::MultiThreadedExecutor executor;
              // executor.add_node(move_group_node);
              // std::thread([&executor]() { executor.spin(); }).detach();
              // static const std::string PLANNING_GROUP = "manipulator";
              // moveit::planning_interface::MoveGroupInterface move_group(move_group_node, PLANNING_GROUP);
              // moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
              // const moveit::core::JointModelGroup* joint_model_group = move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP);
              // moveit::planning_interface::MoveGroupInterface::Plan my_plan;
              //this->move_group.setStartStateToCurrentState();

              auto const LOGGER = rclcpp::get_logger("absoluteMovementQuadCrustbase");
              std::vector<double> joints = {joint1, joint2, joint3, joint4};
              this->move_group.setJointValueTarget(joints);
              bool success = static_cast<bool>(this->move_group.plan(my_plan));
              RCLCPP_INFO(LOGGER, " (joint movement) %s", success ? "" : "FAILED");
              if(success == true){
                  //move_group.move();
                  this->move_group.execute(my_plan);
                  status = true;

                  
              }
              else
              {
                  status = false;
              }
              //executor.cancel();
              return status;
    }

    //
    bool RealativeJointMovement(double joint1, double joint2, double joint3, double joint4){
              bool status;
              // rclcpp::NodeOptions node_options;
              // node_options.automatically_declare_parameters_from_overrides(true);
              // auto move_group_node = rclcpp::Node::make_shared("robot_server_com_node", node_options);
              // rclcpp::executors::MultiThreadedExecutor executor;
              // executor.add_node(move_group_node);
              // std::thread([&executor]() { executor.spin(); }).detach();
              // static const std::string PLANNING_GROUP = "manipulator";
              // moveit::planning_interface::MoveGroupInterface move_group(move_group_node, PLANNING_GROUP);
              // moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
              // const moveit::core::JointModelGroup* joint_model_group = move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP);
              // moveit::planning_interface::MoveGroupInterface::Plan my_plan;
              auto const LOGGER = rclcpp::get_logger("realative joint");
              std::vector<double> joints = {joint1, joint2, joint3, joint4};
              moveit::core::RobotStatePtr current_state = this->move_group.getCurrentState(10);
              current_state->copyJointGroupPositions(joint_model_group, joints);
              joints[0] += joint1;
              joints[1] += joint2;
              joints[2] += joint3;
              joints[3] += joint4;
              this->move_group.setJointValueTarget(joints);
              bool success = static_cast<bool>(this->move_group.plan(my_plan));
              RCLCPP_INFO(LOGGER, " (realative joint movement) %s", success ? "" : "FAILED");
              if(success == true){
                  //move_group.move();
                  this->move_group.execute(my_plan);
                  status = true;

                  
              }
              else
              {
                  status = false;
              }
              //executor.cancel();
              return status;
    }

    bool absoluteMovementQuadCrustBaseWOrientationTolerance(double x, double y, double z, double q_x, double q_y, double q_z, double q_w, double orientation_tolerance)
      {
              bool status;
              // rclcpp::NodeOptions node_options;
              // node_options.automatically_declare_parameters_from_overrides(true);
              // auto move_group_node = rclcpp::Node::make_shared("robot_server_com_node", node_options);
              // rclcpp::executors::MultiThreadedExecutor executor;
              // executor.add_node(move_group_node);
              // std::thread([&executor]() { executor.spin(); }).detach();
              // static const std::string PLANNING_GROUP = "manipulator";
              // moveit::planning_interface::MoveGroupInterface move_group(move_group_node, PLANNING_GROUP);
              // moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
              // const moveit::core::JointModelGroup* joint_model_group = move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP);
              auto const LOGGER = rclcpp::get_logger("absoluteMovementQuadCrustbase");
             
              this->move_group.setNumPlanningAttempts(5);

              this->move_group.setPlanningTime(10.0);
              this->move_group.setGoalOrientationTolerance(orientation_tolerance);

              // moveit::planning_interface::MoveGroupInterface::Plan my_plan;

              this->move_group.setPoseReferenceFrame("crust_base_link");
              geometry_msgs::msg::PoseStamped current_pose;
              
              //this->move_group.setStartStateToCurrentState();
              
              geometry_msgs::msg::Pose pos;
              pos.orientation.x = q_x;
              pos.orientation.y = q_y;
              pos.orientation.z = q_z;
              pos.orientation.w = q_w;
              pos.position.x = x;
              pos.position.y = y;
              pos.position.z = z;
              this->move_group.setPoseTarget(pos);
              bool success = static_cast<bool>(this->move_group.plan(my_plan));
              RCLCPP_INFO(LOGGER, " (absolute qoud base with tolerance) %s", success ? "" : "FAILED");
              if(success == true){
                  //move_group.move();
                  this->move_group.execute(my_plan);
                  status = true;

                  
              }
              else
              {
                  status = false;
              }
              //executor.cancel();
              return status;
      }

    bool absoluteMovementQuadCrustBase(double x, double y, double z, double q_x, double q_y, double q_z, double q_w)
      {
              bool status;
              // rclcpp::NodeOptions node_options;
              // node_options.automatically_declare_parameters_from_overrides(true);
              // auto move_group_node = rclcpp::Node::make_shared("robot_server_com_node", node_options);
              // rclcpp::executors::MultiThreadedExecutor executor;
              // executor.add_node(move_group_node);
              // std::thread([&executor]() { executor.spin(); }).detach();
              // static const std::string PLANNING_GROUP = "manipulator";
              // moveit::planning_interface::MoveGroupInterface move_group(move_group_node, PLANNING_GROUP);
              // moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
              // const moveit::core::JointModelGroup* joint_model_group = move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP);
              auto const LOGGER = rclcpp::get_logger("absoluteMovementQuadCrustbase");
             
              this->move_group.setNumPlanningAttempts(5);

              this->move_group.setPlanningTime(10.0);
              //move_group.setGoalOrientationTolerance(0.5);

              // moveit::planning_interface::MoveGroupInterface::Plan my_plan;

              this->move_group.setPoseReferenceFrame("crust_base_link");
              geometry_msgs::msg::PoseStamped current_pose;
              
              //this->move_group.setStartStateToCurrentState();
              
              geometry_msgs::msg::Pose pos;
              pos.orientation.x = q_x;
              pos.orientation.y = q_y;
              pos.orientation.z = q_z;
              pos.orientation.w = q_w;
              pos.position.x = x;
              pos.position.y = y;
              pos.position.z = z;
              this->move_group.setPoseTarget(pos);
              bool success = static_cast<bool>(this->move_group.plan(my_plan));
              RCLCPP_INFO(LOGGER, " (absoluteMovementQuadCrustBase) %s", success ? "" : "FAILED");
              if(success == true){
                  //move_group.move();
                  this->move_group.execute(my_plan);
                  status = true;

                  
              }
              else
              {
                  status = false;
              }
              //executor.cancel();
              return status;
      }

    bool absoluteMovementRPYCrustBase(double x, double y, double z, double rot_r, double rot_p, double rot_y){
              bool status;
              // rclcpp::NodeOptions node_options;
              // node_options.automatically_declare_parameters_from_overrides(true);
              // auto move_group_node = rclcpp::Node::make_shared("robot_server_com_node", node_options);
              // rclcpp::executors::MultiThreadedExecutor executor;
              // executor.add_node(move_group_node);
              // std::thread([&executor]() { executor.spin(); }).detach();
              // static const std::string PLANNING_GROUP = "manipulator";
              // moveit::planning_interface::MoveGroupInterface move_group(move_group_node, PLANNING_GROUP);
              // moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
              // const moveit::core::JointModelGroup* joint_model_group = move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP);
              auto const LOGGER = rclcpp::get_logger("Relative movement quad rpy");
             
              this->move_group.setNumPlanningAttempts(5);

              this->move_group.setPlanningTime(10.0);
              this->move_group.setGoalOrientationTolerance(0.1);
              //std::vector< double > current_rpy =	move_group.getCurrentRPY();
              // moveit::planning_interface::MoveGroupInterface::Plan my_plan;
             
              
              this->move_group.setPoseReferenceFrame("crust_base_link");
              //geometry_msgs::msg::PoseStamped current_pose;
              //current_pose = move_group.getCurrentPose();
              
              
              //this->move_group.setStartStateToCurrentState();
              geometry_msgs::msg::Pose pos;
              tf2::Quaternion q_rot;
              //q_orig.setRPY(current_rpy[0],current_rpy[1],current_rpy[2]);
              q_rot.setRPY(rot_r, rot_p, rot_y);
              //q_new = q_rot * q_orig;
              q_rot.normalize();
              pos.orientation.x = q_rot.x();
              pos.orientation.y = q_rot.y();
              pos.orientation.z = q_rot.z();
              pos.orientation.w = q_rot.w();
              pos.position.x = x;
              pos.position.y = y;
              pos.position.z = z;
              //move_group.setPositionTarget(x,y,z);
              //move_group.setRPYTarget(rot_r, rot_p, rot_y);
              this->move_group.setPoseTarget(pos);
              
              bool success = static_cast<bool>(this->move_group.plan(my_plan));
              RCLCPP_INFO(LOGGER, " (absoluteMovementRPYCrustBase) %s", success ? "" : "FAILED");
              if(success == true){
                  //move_group.move();
                  move_group.execute(my_plan);
                  status = true;

                  
              }
              else
              {
                  status = false;
              }
              //executor.cancel();
              return status;
    }

    void tester(const std::shared_ptr<crust_msgs::srv::RobotCmdSrv::Request> request,
            std::shared_ptr<crust_msgs::srv::RobotCmdSrv::Response>      response)
            { 

              // rclcpp::NodeOptions node_options;
              // node_options.automatically_declare_parameters_from_overrides(true);
              // auto move_group_node = rclcpp::Node::make_shared("move_group_interface_tutorial", node_options);
              // // For current state monitor
              // rclcpp::executors::MultiThreadedExecutor executor;
              // //rclcpp::executors::SingleThreadedExecutor executor;
              // executor.add_node(move_group_node);
              // std::thread([&executor]() { executor.spin(); }).detach();
              // static const std::string PLANNING_GROUP = "manipulator";
              auto const LOGGER = rclcpp::get_logger("hello_moveit");
              // moveit::planning_interface::MoveGroupInterface move_group(move_group_node, PLANNING_GROUP);
              // moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
              //const moveit::core::JointModelGroup* joint_model_group = move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP);
              
              RCLCPP_INFO(LOGGER, "Planning frame: %s", this->move_group.getPlanningFrame().c_str());

              // We can also print the name of the end-effector link for this group.
              RCLCPP_INFO(LOGGER, "End effector link: %s", this->move_group.getEndEffectorLink().c_str());

              // We can get a list of all the groups in the robot:
              RCLCPP_INFO(LOGGER, "Available Planning Groups:");
              std::copy(move_group.getJointModelGroupNames().begin(), this->move_group.getJointModelGroupNames().end(),std::ostream_iterator<std::string>(std::cout, ", "));
              
              
              response->status =  request->cmd;
              RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Incoming request\na: %ld",
                            request->cmd);
              RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Incoming request\na: %f",
                            request->pose[0]);

              RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "sending back response: [%ld]", (long int)response->status);
              //executor.cancel();
            }

    bool relativeMovementQuad(double x, double y, double z, double q_x, double q_y, double q_z, double q_w){
              bool status;
              
              // auto move_group_node = rclcpp::Node::make_shared("robot_server_com_node");
              // rclcpp::executors::MultiThreadedExecutor executor;
              // executor.add_node(move_group_node);
              // std::thread([&executor]() { executor.spin(); }).detach();
              // static const std::string PLANNING_GROUP = "manipulator";
              // moveit::planning_interface::MoveGroupInterface move_group(move_group_node, PLANNING_GROUP);
              // moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
              // const moveit::core::JointModelGroup* joint_model_group = move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP);
              auto const LOGGER = rclcpp::get_logger("Relative movement quad");
              //const moveit::core::JointModelGroup* joint_model_group = move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP);
    
              this->move_group.setNumPlanningAttempts(5);

              this->move_group.setPlanningTime(10.0);

              // moveit::planning_interface::MoveGroupInterface::Plan my_plan;

              this->move_group.setPoseReferenceFrame("tool_link");
              
              //this->move_group.setStartStateToCurrentState();
              
              geometry_msgs::msg::Pose relative_tool;
              relative_tool.orientation.x = q_x;
              relative_tool.orientation.y = q_y;
              relative_tool.orientation.z = q_z;
              relative_tool.orientation.w = q_w;
              relative_tool.position.x = x;
              relative_tool.position.y = y;
              relative_tool.position.z = z;
              this->move_group.setPoseTarget(relative_tool);
              bool success = static_cast<bool>(this->move_group.plan(my_plan));
              RCLCPP_INFO(LOGGER, " (relativeMovementQuad) %s", success ? "" : "FAILED");
              if(success == true){
                  //move_group.move();
                  this->move_group.execute(my_plan);
                  status = true;

                  
              }
              else
              {
                  status = false;
              }
              //executor.cancel();
              return status;
      }

    bool relativeMovementRPY(double x, double y, double z, double rot_r, double rot_p, double rot_y){
            bool status;
            
            // auto move_group_node = rclcpp::Node::make_shared("robot_server_com_node");
            // rclcpp::executors::MultiThreadedExecutor executor;
            // executor.add_node(move_group_node);
            // std::thread([&executor]() { executor.spin(); }).detach();
            // static const std::string PLANNING_GROUP = "manipulator";
            // moveit::planning_interface::MoveGroupInterface move_group(move_group_node, PLANNING_GROUP);
            // moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
            // const moveit::core::JointModelGroup* joint_model_group = move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP);
            auto const LOGGER = rclcpp::get_logger("Relative movement RPY");
            //const moveit::core::JointModelGroup* joint_model_group = move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP);
            this->move_group.setPoseReferenceFrame("crust_base_link");
            this->move_group.setNumPlanningAttempts(5);

            this->move_group.setPlanningTime(10.0);

            // moveit::planning_interface::MoveGroupInterface::Plan my_plan;

            geometry_msgs::msg::PoseStamped current_pose;
            current_pose = this->move_group.getCurrentPose();
            std::vector< double > current_rpy =	this->move_group.getCurrentRPY();
            tf2::Quaternion q_orig, q_rot, q_new;
            q_orig.setRPY(current_rpy[0],current_rpy[1],current_rpy[2]);
            q_rot.setRPY(rot_r, rot_p, rot_y);
            q_new = q_rot * q_orig;
            q_new.normalize();
            tf2::Vector3 vec_new = tf2::operator*({x,y,x},{current_pose.pose.position.x,current_pose.pose.position.y,current_pose.pose.position.z});
            this->move_group.setPoseReferenceFrame("tool_link");
            
            //this->move_group.setStartStateToCurrentState();
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
          
            this->move_group.setRPYTarget(rot_r,rot_p,rot_y);
            this->move_group.setPositionTarget(x,y,z);
            bool success = static_cast<bool>(this->move_group.plan(my_plan));
            RCLCPP_INFO(LOGGER, " (relativeMovementRPY) %s", success ? "" : "FAILED");
            if(success == true){
                this->move_group.execute(my_plan);
                //move_group.execute(my_plan);
                status = true;

                
            }
            else
            {
                status = false;
            }
            //executor.cancel();
            return status;
  }

    bool baseRelativeMovementQuad(double x, double y, double z, double q_x, double q_y, double q_z, double q_w)
    {
              bool status;
              // rclcpp::NodeOptions node_options;
              // node_options.automatically_declare_parameters_from_overrides(true);
              // auto move_group_node = rclcpp::Node::make_shared("robot_server_com_node", node_options);
              // rclcpp::executors::MultiThreadedExecutor executor;
              // executor.add_node(move_group_node);
              // std::thread([&executor]() { executor.spin(); }).detach();
              // static const std::string PLANNING_GROUP = "manipulator";
              // moveit::planning_interface::MoveGroupInterface move_group(move_group_node, PLANNING_GROUP);
              // moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
              // const moveit::core::JointModelGroup* joint_model_group = move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP);
              auto const LOGGER = rclcpp::get_logger("Base Relative movement quad");
             
              this->move_group.setNumPlanningAttempts(5);

              this->move_group.setPlanningTime(10.0);

              // moveit::planning_interface::MoveGroupInterface::Plan my_plan;

              this->move_group.setPoseReferenceFrame("crust_base_link");
              geometry_msgs::msg::PoseStamped current_pose;
              current_pose = this->move_group.getCurrentPose();
              
              //this->move_group.setStartStateToCurrentState();
              
              geometry_msgs::msg::Pose relative_tool;
              relative_tool.orientation.x = current_pose.pose.orientation.x+q_x;
              relative_tool.orientation.y = current_pose.pose.orientation.y+q_y;
              relative_tool.orientation.z = current_pose.pose.orientation.z+q_z;
              relative_tool.orientation.w = current_pose.pose.orientation.w+q_w;
              relative_tool.position.x = current_pose.pose.position.x+x;
              relative_tool.position.y = current_pose.pose.position.y+y;
              relative_tool.position.z = current_pose.pose.position.z+z;
              this->move_group.setPoseTarget(relative_tool);
              bool success = static_cast<bool>(this->move_group.plan(my_plan));
              RCLCPP_INFO(LOGGER, " (baseRelativeMovementQuad) %s", success ? "" : "FAILED");
              if(success == true){
                  //move_group.move();
                  this->move_group.execute(my_plan);
                  status = true;

                  
              }
              else
              {
                  status = false;
              }
              //executor.cancel();
              return status;
    }

    bool baseRelativeMovementRPY(double x, double y, double z, double rot_r, double rot_p, double rot_y)
    {
              bool status;
              // rclcpp::NodeOptions node_options;
              // node_options.automatically_declare_parameters_from_overrides(true);
              // auto move_group_node = rclcpp::Node::make_shared("robot_server_com_node", node_options);
              // rclcpp::executors::MultiThreadedExecutor executor;
              // executor.add_node(move_group_node);
              // std::thread([&executor]() { executor.spin(); }).detach();
              // static const std::string PLANNING_GROUP = "manipulator";
              // moveit::planning_interface::MoveGroupInterface move_group(move_group_node, PLANNING_GROUP);
              // moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
              // const moveit::core::JointModelGroup* joint_model_group = move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP);
              auto const LOGGER = rclcpp::get_logger("BaseRelative movement RPY");
             
              this->move_group.setNumPlanningAttempts(10);
              this->move_group.setPoseReferenceFrame("crust_base_link");
              this->move_group.setPlanningTime(10.0);
              this->move_group.setGoalOrientationTolerance(0.005);
              this->move_group.setGoalPositionTolerance(0.005);

              //move_group.setMaxVelocityScalingFactor(0.05);
              //move_group.setMaxAccelerationScalingFactor(0.05);
              std::vector< double > current_rpy =	this->move_group.getCurrentRPY();
              // moveit::planning_interface::MoveGroupInterface::Plan my_plan;
              tf2::Quaternion q_orig, q_rot, q_new;
              q_orig.setRPY(current_rpy[0],current_rpy[1],current_rpy[2]);
              q_rot.setRPY(rot_r, rot_p, rot_y);
              q_new = q_rot * q_orig;
              q_new.normalize();


              
              
              geometry_msgs::msg::PoseStamped current_pose;
              current_pose = this->move_group.getCurrentPose();
              
              
              //this->move_group.setStartStateToCurrentState();
              
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
              this->move_group.setPoseTarget(pos);
              bool success = static_cast<bool>(this->move_group.plan(my_plan));
              RCLCPP_INFO(LOGGER, " (baseRelativeMovementRPY) %s", success ? "" : "FAILED");
              if(success == true){
                  //move_group.move();
                  this->move_group.execute(my_plan);
                  status = true;

                  
              }
              else
              {
                  status = false;
              }
              //executor.cancel();
              return status;
    }
  private:
    rclcpp::Service<crust_msgs::srv::RobotCmdSrv>::SharedPtr service;

        // Subscription to pose published by sensor node
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;
    rclcpp::Subscription<crust_msgs::msg::RobotCmdMsg>::SharedPtr cmd_sub_;
    rclcpp::Publisher<std_msgs::msg::Int8>::SharedPtr cmd_pub_;
    // Listener for the broadcast transform message
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    // Buffer that stores several seconds of transforms for easy lookup by the listener
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    // Pose in source frame (`sensor_link`)
    geometry_msgs::msg::PoseStamped pose_in_;
    // Pose in target frame (`arm_end_link`)
    geometry_msgs::msg::PoseStamped pose_out_;

    crust_msgs::msg::RobotCmdMsg robot_msg;

    std_msgs::msg::Int8 status_msg;


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
 // test 
  auto target_follower = std::make_shared<RobotHandler>();
  //rclcpp::executors::SingleThreadedExecutor executor;
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(target_follower);
  executor.spin();
  rclcpp::shutdown();
// test

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

  //RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Server Ready.");
  //rclcpp::spin(std::make_shared<RobotHandler>());
  
  //rclcpp::shutdown();
}