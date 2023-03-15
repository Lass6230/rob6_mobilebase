#include "rclcpp/rclcpp.hpp"
#include "crust_msgs/srv/robot_cmd_srv.hpp"                                       // CHANGE

#include <chrono>
#include <cstdlib>
#include <memory>

using namespace std::chrono_literals;

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);

  
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "usage: robot_cmd_srv_client X Y Z");      // CHANGE
   

  std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("robot_cmd_srv_client");  // CHANGE
  rclcpp::Client<crust_msgs::srv::RobotCmdSrv>::SharedPtr client =                // CHANGE
    node->create_client<crust_msgs::srv::RobotCmdSrv>("robot_cmd_srv");          // CHANGE

  auto request = std::make_shared<crust_msgs::srv::RobotCmdSrv::Request>();       // CHANGE
  request->cmd = 14;
  request->pose = {0.01,0.0,0.0,0.0,0.0,0.0,0.0};
                                                               // CHANGE

  while (!client->wait_for_service(1s)) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service. Exiting.");
      return 0;
    }
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "service not available, waiting again...");
  }

  auto result = client->async_send_request(request);
  // Wait for the result.
  if (rclcpp::spin_until_future_complete(node, result) ==
    rclcpp::FutureReturnCode::SUCCESS)
  {
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Sum: %ld", result.get()->status);
  } else {
    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to call service add_three_ints");    // CHANGE
  }


  request = std::make_shared<crust_msgs::srv::RobotCmdSrv::Request>(); 
  request->cmd = 4;
  request->pose = {0.0,0.01,0.0,0.0,0.0,0.0};
  while (!client->wait_for_service(1s)) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service. Exiting.");
      return 0;
    }
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "service not available, waiting again...");
  }
  result = client->async_send_request(request);
  // Wait for the result.
  if (rclcpp::spin_until_future_complete(node, result) ==
    rclcpp::FutureReturnCode::SUCCESS)
  {
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Sum: %ld", result.get()->status);
  } else {
    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to call service add_three_ints");    // CHANGE
  }

   request = std::make_shared<crust_msgs::srv::RobotCmdSrv::Request>(); 
  request->cmd = 12;
  request->pose = {0.04,0.0,0.0,0.0,0.3,0.0};
  while (!client->wait_for_service(1s)) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service. Exiting.");
      return 0;
    }
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "service not available, waiting again...");
  }
  result = client->async_send_request(request);
  // Wait for the result.
  if (rclcpp::spin_until_future_complete(node, result) ==
    rclcpp::FutureReturnCode::SUCCESS)
  {
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Sum: %ld", result.get()->status);
  } else {
    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to call service add_three_ints");    // CHANGE
  }


  rclcpp::shutdown();
  return 0;
}