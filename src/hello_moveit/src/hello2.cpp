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

#include <stdio.h>


//#include <moveit_visual_tools/moveit_visual_tools.h>

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::NodeOptions node_options;
    node_options.automatically_declare_parameters_from_overrides(true);
    auto move_group_node = rclcpp::Node::make_shared("hello2", node_options);
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(move_group_node);
    std::thread([&executor]() { executor.spin(); }).detach();
    
    static const std::string PLANNING_GROUP = "manipulator";
    moveit::planning_interface::MoveGroupInterface move_group(move_group_node, PLANNING_GROUP);
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
    const moveit::core::JointModelGroup* joint_model_group = move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP);
    auto const LOGGER = rclcpp::get_logger("hello_moveit");

    RCLCPP_INFO(LOGGER, "Planning frame: %s", move_group.getPlanningFrame().c_str());

    // We can also print the name of the end-effector link for this group.
    RCLCPP_INFO(LOGGER, "End effector link: %s", move_group.getEndEffectorLink().c_str());

    // We can get a list of all the groups in the robot:
    RCLCPP_INFO(LOGGER, "Available Planning Groups:");
    std::copy(move_group.getJointModelGroupNames().begin(), move_group.getJointModelGroupNames().end(),std::ostream_iterator<std::string>(std::cout, ", "));

    //Here under we can set the tolerance for pose movement and joint movement
    //move_group.setGoalOrientationTolerance();
    //move_group.setGoalOrientationTolerance();
    //move_group.setGoalPositionTolerance();

   
    RCLCPP_INFO(LOGGER,"planing frame: %s",move_group.getPlanningFrame().c_str());
    move_group.setPoseReferenceFrame("crust_base_link");
    RCLCPP_INFO(LOGGER,"new planing frame: %s",move_group.getPoseReferenceFrame().c_str());

    // geometry_msgs::msg::PoseStamped end_pose;
    // end_pose = move_group.getCurrentPose();

    // geometry_msgs::msg::Pose target_pose1;
    // target_pose1.orientation.w = 1.0;
    // target_pose1.position.x = 0.1;
    // target_pose1.position.y = -0.2;
    // target_pose1.position.z = 0.5;
    // move_group.setPoseTarget(target_pose1);

    // we hear the planner if the plan is possible?
    moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    bool success;
    //moveit::core::MoveItErrorCode::SUCCESS suc;
    // bool success = (move_group.plan(my_plan) == suc);
    // bool success = static_cast<bool>(move_group.plan(my_plan));
    
    // RCLCPP_INFO(LOGGER, "Visualizing plan 1 (pose goal) %s", success ? "" : "FAILED");
    // if(success == true){
    //     //move_group.move();
    //     move_group.execute(my_plan);

    // }

    moveit::core::RobotStatePtr current_state = move_group.getCurrentState(10);
    //
    // Next get the current set of joint values for the group.
    std::vector<double> joint_group_positions;
    current_state->copyJointGroupPositions(joint_model_group, joint_group_positions);

    // Now, let's modify one of the joints, plan to the new joint space goal, and visualize the plan.
    joint_group_positions[0] = -0.5;  // radians
    joint_group_positions[1] = -0.2;
    joint_group_positions[2] = -0.2;
    joint_group_positions[3] = -0.2;
    move_group.setJointValueTarget(joint_group_positions);

    // We lower the allowed maximum velocity and acceleration to 5% of their maximum.
    // The default values are 10% (0.1).
    // Set your preferred defaults in the joint_limits.yaml file of your robot's moveit_config
    // or set explicit factors in your code if you need your robot to move faster.
    move_group.setMaxVelocityScalingFactor(0.05);
    move_group.setMaxAccelerationScalingFactor(0.05);

    success = static_cast<bool>(move_group.plan(my_plan));

    
    RCLCPP_INFO(LOGGER, "Visualizing plan 2 (joint space goal) %s", success ? "" : "FAILED");

    if(success == true){
        //move_group.move();
        move_group.execute(my_plan);
        RCLCPP_INFO(LOGGER, "movement statement plan2 done");
    }

    // test of  move realative
    
//     std::vector<double> robot_position;
//     geometry_msgs::msg::PoseStamped target_pose3;
//     target_pose3 = move_group.getCurrentPose();
//     RCLCPP_INFO(LOGGER,"x: %f", target_pose3.pose.position.x);
//     RCLCPP_INFO(LOGGER,"y: %f", target_pose3.pose.position.y);
//     RCLCPP_INFO(LOGGER,"z: %f", target_pose3.pose.position.z);
//     target_pose3.pose.position.x  += 0.05;
//     target_pose3.pose.position.y -= 0.1;
//     move_group.setPoseTarget(target_pose3);
//     success = static_cast<bool>(move_group.plan(my_plan));
    
//    RCLCPP_INFO(LOGGER, "Relative movement plan 1 (relative pose goal) %s", success ? "" : "FAILED");
//     if(success == true){
//        move_group.move();
//     }

    // move_group.setPoseTarget(end_pose);
    // success = static_cast<bool>(move_group.plan(my_plan));
    
    // RCLCPP_INFO(LOGGER, "Relative movement plan 1 (end pose goal) %s", success ? "" : "FAILED");
    // if(success == true){
    //     move_group.move();
    // }


    //pack robot down
    current_state = move_group.getCurrentState(10);
    std::vector<double> joint_pos_pack_down_robot;
    //current_state->copyJointGroupPositions(joint_model_group, joint_pos_pack_down_robot);
    
    joint_pos_pack_down_robot[0] = 0.0;
    joint_pos_pack_down_robot[1] = -1.5;
    joint_pos_pack_down_robot[2] = 0.1;
    joint_pos_pack_down_robot[3] = 1.5;
    move_group.setJointValueTarget(joint_pos_pack_down_robot);

    move_group.setMaxVelocityScalingFactor(0.05);
    move_group.setMaxAccelerationScalingFactor(0.05);
    success = static_cast<bool>(move_group.plan(my_plan));

    
    RCLCPP_INFO(LOGGER, "Pack down robot %s", success ? "" : "FAILED");

    if(success == true){
        move_group.execute(my_plan);
        RCLCPP_INFO(LOGGER, "movement statement plan3 done");
        //move_group.move();
    }

    // end pack robot down

    rclcpp::shutdown();
    return 0;
}