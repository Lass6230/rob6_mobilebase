#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>

#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/msg/display_robot_state.hpp>
#include <moveit_msgs/msg/display_trajectory.hpp>

#include <moveit_msgs/msg/attached_collision_object.hpp>
#include <moveit_msgs/msg/collision_object.hpp>

#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>

int main(int argc, char * argv[])
{
  // Initialize ROS and create the Node
  rclcpp::init(argc, argv);
  auto const node = std::make_shared<rclcpp::Node>(
    "hello_moveit",
    rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true)
  );

  // Create a ROS logger
  auto const logger = rclcpp::get_logger("hello_moveit");
  static const std::string PLANNING_GROUP = "manipulator";

  // Create the MoveIt MoveGroup Interface
  using moveit::planning_interface::MoveGroupInterface;
  auto move_group_interface = MoveGroupInterface(node, "manipulator");

  geometry_msgs::msg::PoseStamped current_pos = move_group_interface.getPoseTarget("link_4");
  //RCLCPP_ERROR(logger, current_pos);
  RCLCPP_INFO(logger, "Planning frame: %s", move_group_interface.getPlanningFrame().c_str());
  RCLCPP_INFO(logger, "End effector link: %s", move_group_interface.getEndEffectorLink().c_str());
  RCLCPP_INFO(logger, "Available Planning Groups:");
  std::copy(move_group_interface.getJointModelGroupNames().begin(), move_group_interface.getJointModelGroupNames().end(),
          std::ostream_iterator<std::string>(std::cout, ", "));
  
  robot_model_loader::RobotModelLoader robot_model_loader(node);
  const moveit::core::RobotModelPtr& kinematic_model = robot_model_loader.getModel();
  RCLCPP_INFO(logger, "Model frame: %s", kinematic_model->getModelFrame().c_str());
  moveit::core::RobotStatePtr kinematic_state(new moveit::core::RobotState(kinematic_model));
  kinematic_state->setToDefaultValues();
  const moveit::core::JointModelGroup* joint_model_group = kinematic_model->getJointModelGroup("manipulator");

  const std::vector<std::string>& joint_names = joint_model_group->getVariableNames();
  std::vector<double> joint_values;
  kinematic_state->copyJointGroupPositions(joint_model_group, joint_values);
  for (std::size_t i = 0; i < joint_names.size(); ++i)
  {
    RCLCPP_INFO(logger, "Joint %s: %f", joint_names[i].c_str(), joint_values[i]);
  }
  auto const target_pose = []{
    geometry_msgs::msg::Pose msg;
    
 
    msg.orientation.w = 1.0;
    msg.position.x = 0.0078;
    msg.position.y = 0.17;
    msg.position.z = 0.53;
    return msg;
  }();
  move_group_interface.setPoseTarget(target_pose);

  // Create a plan to that target pose
  auto const [success, plan] = [&move_group_interface]{
    moveit::planning_interface::MoveGroupInterface::Plan msg;
    auto const ok = static_cast<bool>(move_group_interface.plan(msg));
    return std::make_pair(ok, msg);
  }();

  // Execute the plan
  if(success) {
    move_group_interface.execute(plan);
  } else {
    RCLCPP_ERROR(logger, "Planing failed!");
  }

  auto const target_pose2 = []{
    geometry_msgs::msg::Pose msg;
    
 
    msg.orientation.w = 1.0;
    msg.position.x = 0.0078;
    msg.position.y = 0.20;
    msg.position.z = 0.40;
    return msg;
  }();
  move_group_interface.setPoseTarget(target_pose2);

  // Create a plan to that target pose
  auto const [success2, plan2] = [&move_group_interface]{
    moveit::planning_interface::MoveGroupInterface::Plan msg;
    auto const ok = static_cast<bool>(move_group_interface.plan(msg));
    return std::make_pair(ok, msg);
  }();

  // Execute the plan
  if(success2) {
    move_group_interface.execute(plan2);
  } else {
    RCLCPP_ERROR(logger, "Planing failed!");
  }
  // Shutdown ROS
 


  rclcpp::shutdown();
  return 0;
}