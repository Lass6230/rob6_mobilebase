

#include "rclcpp/rclcpp.hpp"
#include <memory>

#include <stdio.h>
#include <chrono>
#include <functional>
#include <cstdlib>
#include <future>

//#include "std_msgs/int8.hpp"
#include <std_msgs/msg/int32.hpp>
#include <std_msgs/msg/int8.hpp>
#include <std_msgs/msg/bool.hpp>
#include <thread>
#include <string>
#include "std_msgs/msg/string.hpp"
#include "crust_msgs/srv/robot_cmd_srv.hpp"
#include "crust_msgs/msg/robot_cmd_msg.hpp"
#include <time.h>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include "tf2/exceptions.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "tf2_ros/buffer.h"
#include <vector>

using std::placeholders::_1;
using std::placeholders::_2;


using namespace std::chrono_literals;

class MainProgram : public rclcpp::Node
{
    public:
        MainProgram() : Node("MainProgram")
        {   
            tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
            // Listen to the buffer of transforms
            tf_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf_buffer_);
            m_clock = std::make_shared< rclcpp::Clock >(RCL_SYSTEM_TIME);
            callback_group_subscriber1_ = this->create_callback_group(
                    rclcpp::CallbackGroupType::MutuallyExclusive);
            callback_group_subscriber2_ = this->create_callback_group(
                    rclcpp::CallbackGroupType::MutuallyExclusive);

            auto sub1_opt = rclcpp::SubscriptionOptions();
            sub1_opt.callback_group = callback_group_subscriber1_;
            auto sub2_opt = rclcpp::SubscriptionOptions();
            sub2_opt.callback_group = callback_group_subscriber2_;
            
            timer_ = this->create_wall_timer(100ms, std::bind(&MainProgram::timer_callback, this),callback_group_subscriber2_ );
            //sub_vac_ = this->create_subscription<std_msgs::msg::Int8>("vaccumControl", 10, std::bind(&MainProgram::vacCallback, this, _1), sub1_opt);
            pub_vac_ = this->create_publisher<std_msgs::msg::Int8>("vaccumControl",10);

            pub_robot_ = this->create_publisher<crust_msgs::msg::RobotCmdMsg>("/robot_cmd_msg",10);
            sub_robot_ = this->create_subscription<std_msgs::msg::Int8>("/robot_cmd_status",10,std::bind(&MainProgram::subRobotStatus, this, _1), sub1_opt);
            
            // camera sub and pub
            sub_camera_aruco_ = this->create_subscription<std_msgs::msg::Int8>("/status_aruco",10,std::bind(&MainProgram::subArucoStatus, this, _1), sub1_opt);
            sub_camera_ball_ = this->create_subscription<std_msgs::msg::Int8>("/status_ball",10,std::bind(&MainProgram::subBallStatus, this, _1), sub1_opt);
            pub_camera_aruco_ = this->create_publisher<std_msgs::msg::Bool>("/search_aruco",10);
            pub_camera_ball_ = this->create_publisher<std_msgs::msg::Bool>("/search_golfball",10);

            pub_mobile_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/goal",10);
            sub_mobile_ = this->create_subscription<std_msgs::msg::Int32>("/goal_feedback", 10,std::bind(&MainProgram::subMobileStatus, this, _1), sub1_opt);
            


            

            // INIT poseStamped values for
            tf2::Quaternion quat_trolly[3];

            quat_trolly[0].setRPY(0.0, 0.0, 0.0);
            trolly_pose[0].pose.position.x = 0.0;
            trolly_pose[0].pose.position.y = 0.0;
            trolly_pose[0].pose.orientation = tf2::toMsg(quat_trolly[0]);

            quat_trolly[1].setRPY(0.0, 0.0, 0.0);
            trolly_pose[1].pose.position.x = 0.0;
            trolly_pose[1].pose.position.y = 0.0;
            trolly_pose[1].pose.orientation = tf2::toMsg(quat_trolly[1]);

            quat_trolly[2].setRPY(0.0, 0.0, 0.0);
            trolly_pose[2].pose.position.x = 0.0;
            trolly_pose[2].pose.position.y = 0.0;
            trolly_pose[2].pose.orientation = tf2::toMsg(quat_trolly[2]);

            tf2::Quaternion quat_house[3];

            quat_house[0].setRPY(0.0, 0.0, 0.0);
            house_pose[0].pose.position.x = 0.0;
            house_pose[0].pose.position.y = 0.0;
            house_pose[0].pose.orientation = tf2::toMsg(quat_house[0]);

            quat_house[1].setRPY(0.0, 0.0, 0.0);
            house_pose[1].pose.position.x = 0.0;
            house_pose[1].pose.position.y = 0.0;
            house_pose[1].pose.orientation = tf2::toMsg(quat_house[1]);

            quat_house[2].setRPY(0.0, 0.0, 0.0);
            house_pose[2].pose.position.x = 0.0;
            house_pose[2].pose.position.y = 0.0;
            house_pose[2].pose.orientation = tf2::toMsg(quat_house[2]);
        }
    
    private:
        void subMobileStatus(const std_msgs::msg::Int32 & msg){
            status_mobile = msg.data;
        }

        void subArucoStatus(const std_msgs::msg::Int8 & msg){
            status_aruco = msg.data;
        }

        void subBallStatus(const std_msgs::msg::Int8 & msg){
            status_ball = msg.data;
        }

        void subRobotStatus(const std_msgs::msg::Int8 & msg){
            robot_status = msg.data;
        }
        void vacCallback(const std_msgs::msg::Int8 & msg){
            auto message = std_msgs::msg::String();
            message.data = "vacCallback" + std::to_string(msg.data);
            RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());

        }

        void main_program(){
            while(1){
                auto message = std_msgs::msg::String();
                message.data = "while " + std::to_string(count_);
                RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
                
            }

        }
        void timer_callback(){
            //auto message = std_msgs::msg::String();
            //message.data = "Hello, world! " + std::to_string(count_++);
            //RCLCPP_INFO(this->get_logger(), "timer_callback: '%s'", message.data.c_str());
        

            // robot look out positions
            // long look x:0.23881 Y:0.2679 Z:0.18507 R0.0 P:0.785398163 Y0.0

            switch (sfc)
            {
            case 0:
                // pack robot down
                RCLCPP_INFO(this->get_logger(), "start timer");
                t_time1 = clock();
                m_lastTime1 = m_clock->now().seconds();
                
                sfc = 6000;//6000;//4000;//155;//1100;

                break;
            case 10:
                t_time2 = clock();
                m_lastTime2 = m_clock->now().seconds();

                RCLCPP_INFO(this->get_logger(), "time: %f",(float)((t_time2 -t_time1)*10.0/CLOCKS_PER_SEC));
                RCLCPP_INFO(this->get_logger(), "time2: %f",(m_lastTime2-m_lastTime1));
                if((float)((t_time2 -t_time1)*10.0/CLOCKS_PER_SEC) >3.0){
                    sfc = 20;
                }
                if((m_lastTime2-m_lastTime1)>3.0){
                    sfc = 20;
                }
                break;
            
            case 20:
                RCLCPP_INFO(this->get_logger(), "after time");
                
                vac_msg.data = 2;
                pub_vac_->publish(vac_msg);
                t_time1 = clock();
                m_lastTime1 = m_clock->now().seconds();
                sfc = 30;

                break;
            case 30:
                t_time2 = clock();
                m_lastTime2 = m_clock->now().seconds();
                RCLCPP_INFO(this->get_logger(), "time: %f",(float)((t_time2 -t_time1)*10.0/CLOCKS_PER_SEC));
                RCLCPP_INFO(this->get_logger(), "time2: %f",(m_lastTime2-m_lastTime1));
                if((float)((t_time2 -t_time1)*10.0/CLOCKS_PER_SEC) >3.0){
                    sfc = 40;
                }
                if((m_lastTime2-m_lastTime1)>3.0){
                    sfc = 40;
                }
                break;
            
            case 40:
                vac_msg.data = 0;
                pub_vac_->publish(vac_msg);
                sfc = 50;
                break;
            case 50:
                robot_msg.cmd = 3;
                robot_msg.pose = {0.0,0.0,0.0,0.0,0.0,0.0};
                pub_robot_->publish(robot_msg);
                sfc = 60;
                break;
            
            case 60:
                if(robot_status == 1){
                    robot_status = 0;
                    sfc = 70;
                }
                break;
            case 70:
                RCLCPP_INFO(this->get_logger(), "robot_done");
                robot_msg.cmd = 4;
                robot_msg.pose = {0.0,0.0,0.0,0.0,0.0,0.0};
                pub_robot_->publish(robot_msg);
                sfc = 80;
                break;
            case 80:
                if(robot_status == 1){
                    robot_status = 0;
                    sfc = 110;
                }
                break;
            
            case 90:
                RCLCPP_INFO(this->get_logger(), "robot_15");
                robot_msg.cmd = 15;
                robot_msg.pose = {0.0,0.0,0.0,0.0,0.0,0.0};
                pub_robot_->publish(robot_msg);
                t_time1 = clock();
                sfc = 100;
                break;
            
            case 100:
                t_time2 = clock();
                if((float)((t_time2 -t_time1)*10.0/CLOCKS_PER_SEC) >5.0){
                    sfc = 110;
                }
                if(robot_status == 1){
                    robot_status = 0;
                    sfc = 110;
                }
                break;
            
            case 110:
                t_time2 = clock();
                vac_msg.data = 2;
                pub_vac_->publish(vac_msg);
                RCLCPP_INFO(this->get_logger(), "robot_16");
                robot_msg.cmd = 16;
                robot_msg.pose = {0.0,0.0,0.0,0.0,0.0,0.0};
                pub_robot_->publish(robot_msg);
                t_time1 = clock();
                sfc = 120;
                break;
            
            case 120:
                t_time2 = clock();
                if(robot_status == 1){
                    robot_status = 0;
                    sfc = 130;
                }
                if((float)((t_time2 -t_time1)*10.0/CLOCKS_PER_SEC) >10.0){
                    sfc = 150;
                    RCLCPP_INFO(this->get_logger(), "timed out");
                    
                }
                break;
            
            case 130:
                RCLCPP_INFO(this->get_logger(), "robot_15");
                vac_msg.data = 0;
                pub_vac_->publish(vac_msg);

                //robot_msg.cmd = 16;
                //robot_msg.pose = {0.0,0.0,0.0,0.0,0.0,0.0};
                //pub_robot_->publish(robot_msg);
                t_time1 = clock();
                sfc = 140;
                break;
            
            case 140:
                t_time2 = clock();
                if(robot_status == 1){
                    robot_status = 0;
                    sfc = 150;
                }
                if((float)((t_time2 -t_time1)*10.0/CLOCKS_PER_SEC) >10.0){
                    RCLCPP_INFO(this->get_logger(), "timed out");
                    sfc = 150;
                }
                break;
            
            case 150:
                RCLCPP_INFO(this->get_logger(), "restarting program");
                //sfc = 0;
                break;

            
            case 155:/// start test af gripper
                robot_msg.cmd = 18; // set gripper off
                robot_msg.pose = {0.37336,-0.007814,0.24958,0.0,1.57,0.0};
                pub_robot_->publish(robot_msg); // send to robot to set gripper off
                sfc = 156;
                break;
            
            case 156:
                if(robot_status == 1){
                    robot_status = 0;
                    robot_attempts = 0;
                    sfc = 157;
                }

                break;
            
            case 157:
                robot_msg.cmd = 19; // set gripper on
                robot_msg.pose = {0.37336,-0.007814,0.24958,0.0,1.57,0.0};
                pub_robot_->publish(robot_msg); // send to robot to set gripper on
                sfc = 158;
                break;
            
            case 158:
                if(robot_status == 1){
                    robot_status = 0;
                    robot_attempts = 0;
                    sfc = 160;
                }

                break;
            
            case 159:

                break;

            case 160: // start of vision, vac and robot test and set both ball and aruco shearch off
                vac_msg.data = 1;// set vaccum on
                pub_vac_->publish(vac_msg);// Turn on vaccum pup

                aruco_msg.data = false;
                pub_camera_aruco_->publish(aruco_msg);
                ball_msg.data = false;
                pub_camera_ball_->publish(aruco_msg);
                status_aruco = 0;
                status_ball = 0;
                
                robot_msg.cmd = 6;
                robot_msg.pose = {0.34,0.0,0.13129,0.0,1.4,0.0};//{0.37336,-0.007814,0.24958,0.0,1.57,0.0};
                pub_robot_->publish(robot_msg);// make the robot go to the defalut pos
                m_lastTime1 = m_clock->now().seconds();
                sfc = 170;
                break;

                // function should be look long 45 degrees down for the left middle and right, then look close for left middle right, have a timeout on each to decide when we are done looking
                
            
            case 170: // check if robot default pos have en reached
                m_lastTime2 = m_clock->now().seconds();
                
                if((m_lastTime2-m_lastTime1) >10.0){
                    RCLCPP_INFO(this->get_logger(), "timed out");
                    robot_attempts ++;
                    sfc = 160;
                    if(robot_attempts == 5){
                        sfc = 1000;
                    }
                }
                if(robot_status == 1){
                    robot_status = 0;
                    robot_attempts = 0;
                    sfc = 180;
                }
                break;
            case 180: // Start the aruco shearch
                ball_msg.data = false;
                pub_camera_ball_->publish(ball_msg);
                aruco_msg.data = true;
                pub_camera_aruco_->publish(aruco_msg);

                sfc = 190;
                break;
            
            case 190:
                if(status_ball == 1)
                {
                    status_ball = 0;
                    sfc = 200;
                }
                if(status_aruco == 1)
                {
                    status_aruco = 0;
                    sfc = 200;
                }

                break;
            
            case 200:
                robot_msg.cmd = 15; // go to aruco
                robot_msg.pose = {0.0,0.0,0.0,0.0,0.0,0.0};
                pub_robot_->publish(robot_msg);
                ball_msg.data = false;
                pub_camera_ball_->publish(aruco_msg);
                aruco_msg.data = false;
                pub_camera_aruco_->publish(aruco_msg);
                status_ball = 0;
                m_lastTime1 = m_clock->now().seconds();
                sfc = 210;
                
                break;
            
            case 210:
                 m_lastTime2 = m_clock->now().seconds();
                if(robot_status == 1){
                    robot_status = 0;
                    sfc = 211;
                }
                if((m_lastTime2-m_lastTime1) >15.0){
                    RCLCPP_INFO(this->get_logger(), "timed out");
                    robot_attempts ++;
                    sfc = 200;
                    if(robot_attempts == 5){
                        sfc = 1000;
                    }
                }
                

                break;
            
            case 211:
                robot_msg.cmd = 18; // set gripper off
                robot_msg.pose = {0.37336,-0.007814,0.24958,0.0,1.57,0.0};
                pub_robot_->publish(robot_msg); // send to robot to set gripper off

                
                sfc = 212;
                break;
            
            case 212:
                if(robot_status == 1){
                    robot_status = 0;
                    robot_attempts = 0;
                    sfc = 220;
                }

                break;

                
            
            case 220:
                //vac_msg.data = 0;
                //pub_vac_->publish(vac_msg);
                robot_msg.cmd = 6;
                robot_msg.pose = {0.32873,0.026232,0.09716,0.0,0.57,0.0};//{0.3,-0.007814,0.2958,0.0,1.57,0.0};
                pub_robot_->publish(robot_msg);// make the robot go to the defalut pos
                m_lastTime1 = m_clock->now().seconds();
                sfc = 230;
                break;

            case 230:
                 m_lastTime2 = m_clock->now().seconds();
                if(robot_status == 1){
                    robot_status = 0;
                    sfc = 240;
                }
                if((m_lastTime2-m_lastTime1) >10.0){
                    RCLCPP_INFO(this->get_logger(), "timed out");
                    robot_attempts ++;
                    sfc = 220;
                    if(robot_attempts == 5){
                        sfc = 1000;
                    }
                }
                break;
            case 240:
                sfc = 1000;
                break;

            case 1000:
                vac_msg.data = 0;
                pub_vac_->publish(vac_msg);
                break;





            /// look ball move mobile platform
            case 1100: // Open gripper and set both detectors off
                robot_msg.cmd = 19; // set gripper off
                robot_msg.pose = {0.37336,-0.007814,0.24958,0.0,1.57,0.0};
                pub_robot_->publish(robot_msg); // send to robot to set gripper off
                
                aruco_msg.data = false; // setting both detectots off
                pub_camera_aruco_->publish(aruco_msg); // setting both detectots off
                ball_msg.data = false; // setting both detectots off
                pub_camera_ball_->publish(aruco_msg); // setting both detectots off
                status_aruco = 0; // setting detecttor status to 0
                status_ball = 0; // setting detecttor status to 0

                sfc = 1110;
                break;

            case 1110: // Wait for gripper to be open
                if(robot_status == 1){// Wait for gripper to be open
                    robot_status = 0;
                    robot_attempts = 0;
                    sfc = 1120;
                }
                break;
            
            case 1120: // give command for robot to go to position
                robot_msg.cmd = 6;
                robot_msg.pose = {0.34,0.0,0.13129,0.0,1.4,0.0};//{0.37336,-0.007814,0.24958,0.0,1.57,0.0};
                pub_robot_->publish(robot_msg);// make the robot go to the defalut pos
                m_lastTime1 = m_clock->now().seconds();
                sfc = 1130;
                break;

            case 1130: // waitting for robot to go to position
                 m_lastTime2 = m_clock->now().seconds();
                if(robot_status == 1){
                    robot_status = 0;
                    robot_attempts = 0;
                    sfc = 1140;
                }
                if((m_lastTime2-m_lastTime1) >15.0){
                    RCLCPP_INFO(this->get_logger(), "timed out");
                    robot_attempts ++;
                    sfc = 1120;
                    if(robot_attempts == 5){
                        sfc = 1000;
                    }
                }
                break;

            case 1140: // give command to ball detector to begin
                ball_msg.data = true;
                pub_camera_ball_->publish(ball_msg);
                sfc = 1150;
                m_lastTime1 = m_clock->now().seconds(); // start timer for timeout
                break;
            
            case 1150: // Waitting for ball detetor to detect
                if(status_ball == 1)
                {
                    status_ball = 0;
                    sfc = 1160;
                }
                m_lastTime2 = m_clock->now().seconds();
                if((m_lastTime2-m_lastTime1) >3.0){
                    RCLCPP_INFO(this->get_logger(), "timed out");
                    
                    sfc = 1000;
                    
                }
                break;

            case 1160:
                transform_pose = tf_buffer_->lookupTransform(
                    "workspace_center", "ball",
                    tf2::TimePointZero);
                sfc = 1170;
                break;

            case 1170:
                if(sqrt(pow(transform_pose.transform.translation.x,2)+pow(transform_pose.transform.translation.y,2))<0.1){
                    // the ball are in range
                }
                else{
                    sfc = 1180;
                }
                break;
            
            case 1180:
                target_pose.pose.position.x = transform_pose.transform.translation.x;
                target_pose.pose.position.y = transform_pose.transform.translation.y;
                target_pose.pose.orientation.x = transform_pose.transform.rotation.x;
                target_pose.pose.orientation.y = transform_pose.transform.rotation.y;
                target_pose.pose.orientation.z = transform_pose.transform.rotation.z;
                target_pose.pose.orientation.w = transform_pose.transform.rotation.w;
                pub_mobile_->publish(target_pose);
                // Int8 1(reached) 2(failed) (topic: /goal_reached)
                break;
            

            //////////////////BEGINING Start first gate (3) from case 2000-2999 /////////////////////////////////////////////////////
            ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
            ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

            case 2000:
                sfc = 2010;
                break;
            
            case 2010: // close gripper
                robot_msg.cmd = 18; // close gripper
                robot_msg.pose = {-0.3,-0.3};
                pub_robot_->publish(robot_msg); // send to robot to set gripper close

                m_lastTime1 = m_clock->now().seconds(); // start timer for timeout
                sfc = 2020;
                break;

            case 2020: // waitting for gripper to close
                if(robot_status == 1){ // check if robot is done
                    robot_status = 0;
                    robot_attempts = 0;
                    sfc = 2030;
                }
                m_lastTime2 = m_clock->now().seconds(); // get time now
                if((m_lastTime2-m_lastTime1) >15.0){ // if timeout 
                    RCLCPP_INFO(this->get_logger(), "timed out");
                    robot_attempts ++;
                    
                    sfc = 2010; // go back and resend the robot cmd
                    if(robot_attempts == 5){
                       
                       RCLCPP_INFO(this->get_logger(), "timed out, robot can't go to position");

                    }
                    
                }
                break;
            
            case 2030: // send command to pack down robot
                robot_msg.cmd = 3;
                robot_msg.pose = {0.0,0.0,0.0,0.0,0.0,0.0};
                pub_robot_->publish(robot_msg);
                sfc = 2040;
                break;
            
            case 2040: // wait for robot to be packed down
                if(robot_status == 1){ // check if robot is done
                    robot_status = 0;
                    robot_attempts = 0;
                    sfc = 2050;
                }
                m_lastTime2 = m_clock->now().seconds(); // get time now
                if((m_lastTime2-m_lastTime1) >15.0){ // if timeout 
                    RCLCPP_INFO(this->get_logger(), "timed out");
                    robot_attempts ++;
                    
                    sfc = 2030; // go back and resend the robot cmd
                    if(robot_attempts == 5){
                       
                       RCLCPP_INFO(this->get_logger(), "timed out, robot can't go to position");

                    }
                    
                }
                break;
            
            case 2050: // start line follower

                break;
            
            case 2060: // waitting for line follower to be done

                break;
            
            case 2070: // set robot down in front of the mobile base ready for the ball releaser
                robot_msg.cmd = 6;
                robot_msg.pose = {0.29200,0.0,-0.12,0.0,1.45,0.0};
                pub_robot_->publish(robot_msg);
                m_lastTime1 = m_clock->now().seconds(); // start timer for timeout

                sfc = 2080;
                break;
            
            case 2080: // waitting for robot to go to pose
                if(robot_status == 1){ // check if robot is done
                    robot_status = 0;
                    robot_attempts = 0;
                    sfc = 2090;
                }
                m_lastTime2 = m_clock->now().seconds(); // get time now
                if((m_lastTime2-m_lastTime1) >15.0){ // if timeout 
                    RCLCPP_INFO(this->get_logger(), "timed out");
                    robot_attempts ++;
                    
                    sfc = 2070; // go back and resend the robot cmd
                    if(robot_attempts == 5){
                       
                       RCLCPP_INFO(this->get_logger(), "timed out, robot can't go to position");

                    }
                    
                }
                break;
            
            case 2090:

                break;
            
            case 2100:

                break;
            
            case 2110:

                break;
            
            case 2120:

                break;
            
            case 2130:

                break;

            case 2140:

                break;
            
            case 2150:

                break;

            //////////////////END Start first gate (3) from case 2000-2999 /////////////////////////////////////////////////////
            ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
            ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////




            //////////////////BEGINING TASK 12 from case 6000-6999 /////////////////////////////////////////////////////////////////
            ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
            ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

            case 4000:
                sfc = 4010;
                break;

            case 4010: // open gripper
                robot_msg.cmd = 19; // set gripper off and setting the cameras off
                robot_msg.pose = {0.37336,-0.007814,0.24958,0.0,1.57,0.0}; // pos do not matter
                pub_robot_->publish(robot_msg); // send to robot to set gripper off

                aruco_msg.data = false; 
                pub_camera_aruco_->publish(aruco_msg); // setting the aruco camera off
                ball_msg.data = false;
                pub_camera_ball_->publish(aruco_msg); // setting the ball camera off
                status_aruco = 0;
                status_ball = 0;

                m_lastTime1 = m_clock->now().seconds(); // start timer for timeout
                sfc = 4020;

                break;
            
            case 4020: // waitting for gripper to open
                if(robot_status == 1){// Wait for gripper to be open
                    robot_status = 0;
                    robot_attempts = 0;
                    sfc = 4030;
                }
                m_lastTime2 = m_clock->now().seconds(); // get time now
                if((m_lastTime2-m_lastTime1) >5.0){ // if timeout 
                    RCLCPP_INFO(this->get_logger(), "timed out");
                    
                    sfc = 4030; // go back and resend the robot cmd
                    
                }
                break;

            case 4030: // set pose for robot søg ned lige foran
                robot_msg.cmd = 6;
                robot_msg.pose = {0.34,0.0,0.168,0.0,1.45,0.0};
                pub_robot_->publish(robot_msg);
                m_lastTime1 = m_clock->now().seconds(); // start timer for timeout

                sfc = 4040;

                break;

            case 4040: // waitting for robot to go to pose
                if(robot_status == 1){
                    robot_status = 0;
                    robot_attempts = 0;
                    sfc = 4050;
                }
                m_lastTime2 = m_clock->now().seconds(); // get time now
                if((m_lastTime2-m_lastTime1) >15.0){ // if timeout 
                    RCLCPP_INFO(this->get_logger(), "timed out");
                    robot_attempts ++;
                    
                    sfc = 4030; // go back and resend the robot cmd
                    if(robot_attempts == 5){
                       // sfc = 6030;
                       RCLCPP_INFO(this->get_logger(), "timed out, robot can't go to position");

                    }
                    
                }
                break;
            
            case 4050: // start søgning
                ball_msg.data = true;
                pub_camera_ball_->publish(ball_msg); // setting the ball camera off
                status_ball = 0;

                m_lastTime1 = m_clock->now().seconds(); // start timer for timeout
                
                sfc = 4060;
                break;

            case 4060: // waiiting for ball  IF timeout jump to 4500
                if(status_ball == 1)
                {
                    status_ball = 0;
                    sfc = 4500;
                }
                m_lastTime2 = m_clock->now().seconds(); // get time now
                if((m_lastTime2-m_lastTime1) >5.0){ // if timeout 
                    RCLCPP_INFO(this->get_logger(), "timed out, No golfball found");
                    ball_msg.data = false;
                    pub_camera_ball_->publish(aruco_msg); // setting the ball camera off
                   
                    status_ball = 0;
                    
                    sfc = 4070;
                    
                }

                break;
            
            case 4070: // send to robot to go to pose right looking down
                robot_msg.cmd = 6;
                robot_msg.pose = {0.23319,-0.24479,0.15955,0.0,1.45,-0.785398};
                pub_robot_->publish(robot_msg);
                m_lastTime1 = m_clock->now().seconds(); // start timer for timeout

                sfc = 4080;
                break;

            case 4080:
                if(robot_status == 1){
                    robot_status = 0;
                    robot_attempts = 0;
                    sfc = 4090;
                }
                m_lastTime2 = m_clock->now().seconds(); // get time now
                if((m_lastTime2-m_lastTime1) >15.0){ // if timeout 
                    RCLCPP_INFO(this->get_logger(), "timed out");
                    robot_attempts ++;
                    
                    sfc = 4070; // go back and resend the robot cmd
                    if(robot_attempts == 5){
                       // sfc = 6030;
                       RCLCPP_INFO(this->get_logger(), "timed out, robot can't go to position");

                    }
                    
                }
                break;
            
            case 4090:// start søgning
                ball_msg.data = true;
                pub_camera_ball_->publish(ball_msg); // setting the ball camera off
                status_ball = 0;
                sfc = 4100;

                break;
            
            case 4100: // søgning       if found jump to 4500
                if(status_ball == 1)
                {
                    status_ball = 0;
                    sfc = 4500;
                }
                m_lastTime2 = m_clock->now().seconds(); // get time now
                if((m_lastTime2-m_lastTime1) >5.0){ // if timeout 
                    RCLCPP_INFO(this->get_logger(), "timed out, No golfball found");
                    ball_msg.data = false;
                    pub_camera_ball_->publish(aruco_msg); // setting the ball camera off
                   
                    status_ball = 0;
                    
                    sfc = 4110;
                    
                }

                break;
            
            case 4110: // set robot pose to look down left
                robot_msg.cmd = 6;
                robot_msg.pose = {0.23319,0.24479,0.15955,0.0,1.45,0.785398};
                pub_robot_->publish(robot_msg);
                m_lastTime1 = m_clock->now().seconds(); // start timer for timeout

                sfc = 4120;

                break;
            
            case 4120: // waiting for robot to move to pose
                if(robot_status == 1){
                    robot_status = 0;
                    robot_attempts = 0;
                    sfc = 4130;
                }
                m_lastTime2 = m_clock->now().seconds(); // get time now
                if((m_lastTime2-m_lastTime1) >15.0){ // if timeout 
                    RCLCPP_INFO(this->get_logger(), "timed out");
                    robot_attempts ++;
                    
                    sfc = 4110; // go back and resend the robot cmd
                    if(robot_attempts == 5){
                       // sfc = 6030;
                       RCLCPP_INFO(this->get_logger(), "timed out, robot can't go to position");

                    }
                    
                }

                break;

            case 4130: // start søgning
                ball_msg.data = true;
                pub_camera_ball_->publish(ball_msg); // setting the ball camera off
                status_ball = 0;

                m_lastTime1 = m_clock->now().seconds(); // start timer for timeout
                
                sfc = 4140;
                break;
            
            case 4140: // søgning
                if(status_ball == 1)
                {
                    status_ball = 0;
                    sfc = 4500;
                }
                m_lastTime2 = m_clock->now().seconds(); // get time now
                if((m_lastTime2-m_lastTime1) >5.0){ // if timeout 
                    RCLCPP_INFO(this->get_logger(), "timed out, No golfball found");
                    ball_msg.data = false;
                    pub_camera_ball_->publish(aruco_msg); // setting the ball camera off
                   
                    status_ball = 0;
                    
                    sfc = 4150;
                    
                }
        
                break;

            case 4150: // send command to robot to look long midt
                robot_msg.cmd = 6;
                robot_msg.pose = {0.3430,0.0,0.15955,0.0,0.785398,0.0};
                pub_robot_->publish(robot_msg);
                m_lastTime1 = m_clock->now().seconds(); // start timer for timeout

                sfc = 4160;
                break;  
            
            case 4160: // waitting for robot to go to pose
                if(robot_status == 1){
                    robot_status = 0;
                    robot_attempts = 0;
                    sfc = 4170;
                }
                m_lastTime2 = m_clock->now().seconds(); // get time now
                if((m_lastTime2-m_lastTime1) >15.0){ // if timeout 
                    RCLCPP_INFO(this->get_logger(), "timed out");
                    robot_attempts ++;
                    
                    sfc = 4150; // go back and resend the robot cmd
                    if(robot_attempts == 5){
                       // sfc = 6030;
                       RCLCPP_INFO(this->get_logger(), "timed out, robot can't go to position");

                    }
                    
                }

                break;

            case 4170:
                ball_msg.data = true;
                pub_camera_ball_->publish(ball_msg); // setting the ball camera off
                status_ball = 0;

                m_lastTime1 = m_clock->now().seconds(); // start timer for timeout
                
                sfc = 4180;

                break;
            
            case 4180:
                if(status_ball == 1)
                {
                    status_ball = 0;
                    sfc = 4500;
                }
                m_lastTime2 = m_clock->now().seconds(); // get time now
                if((m_lastTime2-m_lastTime1) >5.0){ // if timeout 
                    RCLCPP_INFO(this->get_logger(), "timed out, No golfball found");
                    ball_msg.data = false;
                    pub_camera_ball_->publish(aruco_msg); // setting the ball camera off
                   
                    status_ball = 0;
                    
                    sfc = 4190;
                    
                }

                break;

            case 4190:// robot go to look long right
                robot_msg.cmd = 21;
                robot_msg.pose = {-0.785398,0.0,0.0,0.0};
                pub_robot_->publish(robot_msg);
                m_lastTime1 = m_clock->now().seconds(); // start timer for timeout

                sfc = 4200;

                break;
            
            case 4200: // waitting for robot to go to pose
                if(robot_status == 1){
                    robot_status = 0;
                    robot_attempts = 0;
                    sfc = 4210;
                }
                m_lastTime2 = m_clock->now().seconds(); // get time now
                if((m_lastTime2-m_lastTime1) >15.0){ // if timeout 
                    RCLCPP_INFO(this->get_logger(), "timed out");
                    robot_attempts ++;
                    
                    sfc = 4190; // go back and resend the robot cmd
                    if(robot_attempts == 5){
                       // sfc = 6030;
                       RCLCPP_INFO(this->get_logger(), "timed out, robot can't go to position");

                    }
                    
                }
                break;
            
            case 4210: // send command to søg
                ball_msg.data = true;
                pub_camera_ball_->publish(ball_msg); // setting the ball camera off
                status_ball = 0;

                m_lastTime1 = m_clock->now().seconds(); // start timer for timeout
                
                sfc = 4230;
                break;
                
            case 4230: // søgning
                if(status_ball == 1)
                {
                    status_ball = 0;
                    sfc = 4500;
                }
                m_lastTime2 = m_clock->now().seconds(); // get time now
                if((m_lastTime2-m_lastTime1) >5.0){ // if timeout 
                    RCLCPP_INFO(this->get_logger(), "timed out, No golfball found");
                    ball_msg.data = false;
                    pub_camera_ball_->publish(aruco_msg); // setting the ball camera off
                   
                    status_ball = 0;
                    
                    sfc = 4240;
                    
                }
                break;
            
            case 4240: // robot to go to pose look long left
                robot_msg.cmd = 21;
                robot_msg.pose = {1.57,0.0,0.0,0.0};
                pub_robot_->publish(robot_msg);
                m_lastTime1 = m_clock->now().seconds(); // start timer for timeout

                sfc = 4250;


                break;
            
            case 4250: // waitting for robot to go to pose
                if(robot_status == 1){
                    robot_status = 0;
                    robot_attempts = 0;
                    sfc = 4260;
                }
                m_lastTime2 = m_clock->now().seconds(); // get time now
                if((m_lastTime2-m_lastTime1) >15.0){ // if timeout 
                    RCLCPP_INFO(this->get_logger(), "timed out");
                    robot_attempts ++;
                    
                    sfc = 4240; // go back and resend the robot cmd
                    if(robot_attempts == 5){
                       // sfc = 6030;
                       RCLCPP_INFO(this->get_logger(), "timed out, robot can't go to position");

                    }
                    
                }

                break;
            
            case 4260: // send command to søg
                ball_msg.data = true;
                pub_camera_ball_->publish(ball_msg); // setting the ball camera off
                status_ball = 0;

                m_lastTime1 = m_clock->now().seconds(); // start timer for timeout
                
                sfc = 4270;
                break;
            
            case 4270:
                if(status_ball == 1)
                {
                    status_ball = 0;
                    sfc = 4500;
                }
                m_lastTime2 = m_clock->now().seconds(); // get time now
                if((m_lastTime2-m_lastTime1) >5.0){ // if timeout 
                    RCLCPP_INFO(this->get_logger(), "timed out, No golfball found");
                    ball_msg.data = false;
                    pub_camera_ball_->publish(aruco_msg); // setting the ball camera off
                   
                    status_ball = 0;
                    
                    sfc = 4260;
                    
                }

                break;
            
            case 4280:

                break;

            case 4290:

                break;
            
            case 4300:

                break;


            case 4500: // ball found

                 break;




            //////////////////END TASK 12 from case 6000-6999 /////////////////////////////////////////////////////////////////
            ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
            ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////





            //////////////////BEGINING Drive to  13 from case 5000-5999 ////////////////////////////////////////////////////////////
            ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
            ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////




            //////////////////END Drive to  13 from case 5000-5999 ////////////////////////////////////////////////////////////
            ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
            ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////




            

            //////////////////BEGINING TASK 13 from case 6000-6999 /////////////////////////////////////////////////////////////////
            ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
            ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
            case 6000:
                sfc = 6010;
                break;
            
            case 6010: // Open Gripper

                robot_msg.cmd = 19; // set gripper off and setting the cameras off
                robot_msg.pose = {0.37336,-0.007814,0.24958,0.0,1.57,0.0}; // pos do not matter
                pub_robot_->publish(robot_msg); // send to robot to set gripper off

                aruco_msg.data = false; 
                pub_camera_aruco_->publish(aruco_msg); // setting the aruco camera off
                ball_msg.data = false;
                pub_camera_ball_->publish(aruco_msg); // setting the ball camera off
                status_aruco = 0;
                status_ball = 0;

                m_lastTime1 = m_clock->now().seconds(); // start timer for timeout
                sfc = 6020;
                break;
            
            case 6020: // Wait for gripper to be open
                if(robot_status == 1){// Wait for gripper to be open
                    robot_status = 0;
                    robot_attempts = 0;
                    sfc = 6030;
                }
                m_lastTime2 = m_clock->now().seconds(); // get time now
                if((m_lastTime2-m_lastTime1) >5.0){ // if timeout 
                    RCLCPP_INFO(this->get_logger(), "timed out");
                    
                    sfc = 6010; // go back and resend the robot cmd
                    
                }

                break;
            
            case 6030: // put the robot in sharch pose
                robot_msg.cmd = 6;
                robot_msg.pose = {0.34,0.0,0.168,0.0,1.45,0.0};
                pub_robot_->publish(robot_msg);
                m_lastTime1 = m_clock->now().seconds(); // start timer for timeout

                sfc = 6040;
                break;
            
            case 6040: // waitting for robot to go to pose
                if(robot_status == 1){
                    robot_status = 0;
                    robot_attempts = 0;
                    sfc = 6050;
                }
                m_lastTime2 = m_clock->now().seconds(); // get time now
                if((m_lastTime2-m_lastTime1) >15.0){ // if timeout 
                    RCLCPP_INFO(this->get_logger(), "timed out");
                    robot_attempts ++;
                    
                    sfc = 6030; // go back and resend the robot cmd
                    if(robot_attempts == 5){
                       // sfc = 6030;
                       RCLCPP_INFO(this->get_logger(), "timed out, robot can't go to position");

                    }
                    
                }

                break;
            
            case 6050: // start the aruco camera
                aruco_msg.data = true;
                pub_camera_aruco_->publish(aruco_msg);

                m_lastTime1 = m_clock->now().seconds(); // start timer for timeout
                sfc = 6060;

                break;
            
            case 6060: // waitting for camera to detect aruco code
                if(status_aruco == 5|| status_aruco == 6 || status_aruco == 20) // check if aruco code is found for rød or green or yellow
                {
                    // the color code is store in the variable status_aruco
                    sfc = 6070;
                }
               
                if((m_lastTime2-m_lastTime1) >5.0){ // check if timeout 
                    RCLCPP_INFO(this->get_logger(), "timed out, no aruco code found");
                    
                    
                }

                break;
            
            case 6070: // move robot to aruco code
                robot_msg.cmd = 15; // go to aruco
                robot_msg.pose = {0.0,0.0,0.0,0.0,0.0,0.0};
                pub_robot_->publish(robot_msg);

                aruco_msg.data = false;
                pub_camera_aruco_->publish(aruco_msg); // send to camera to stop

                m_lastTime1 = m_clock->now().seconds(); // start timer for timeout

                sfc = 6080;
                break;
            
            case 6080: // waitting for robot to go to pose
                if(robot_status == 1){ // check if robot is done
                    robot_status = 0;
                    robot_attempts = 0;
                    sfc = 6090;
                }
                m_lastTime2 = m_clock->now().seconds(); // get time now
                if((m_lastTime2-m_lastTime1) >15.0){ // if timeout 
                    RCLCPP_INFO(this->get_logger(), "timed out");
                    robot_attempts ++;
                    
                    sfc = 6050; // go back and resend the robot cmd
                    if(robot_attempts == 5){
                       // sfc = 6030;
                       RCLCPP_INFO(this->get_logger(), "timed out, robot can't go to position");

                    }
                    
                }

                break;
            
            case 6090: // close gripper
                robot_msg.cmd = 18; // close gripper
                robot_msg.pose = {-0.3,-0.3};
                pub_robot_->publish(robot_msg); // send to robot to set gripper close

                m_lastTime1 = m_clock->now().seconds(); // start timer for timeout

                sfc = 6100;

                break;
            
            case 6100: // waitting for gripper to close
                if(robot_status == 1){ // check if robot is done
                    robot_status = 0;
                    robot_attempts = 0;
                    sfc = 6110;
                }
                m_lastTime2 = m_clock->now().seconds(); // get time now
                if((m_lastTime2-m_lastTime1) >15.0){ // if timeout 
                    RCLCPP_INFO(this->get_logger(), "timed out");
                    robot_attempts ++;
                    
                    sfc = 6090; // go back and resend the robot cmd
                    if(robot_attempts == 5){
                       // sfc = 6030;
                       RCLCPP_INFO(this->get_logger(), "timed out, robot can't go to position");

                    }
                    
                }

                break;
            
            case 6110: // move robot to pose to deapproch pose
                robot_msg.cmd = 6;
                robot_msg.pose = {0.34,0.0,0.13129,0.0,1.4,0.0}; 
                pub_robot_->publish(robot_msg);

                m_lastTime1 = m_clock->now().seconds(); // start timer for timeout

                sfc = 6120;

                break;
            
            case 6120: // waitting for robot to move to pose
                if(robot_status == 1){ // check if robot is done
                    robot_status = 0;
                    robot_attempts = 0;
                    sfc = 6130;
                }
                m_lastTime2 = m_clock->now().seconds(); // get time now
                if((m_lastTime2-m_lastTime1) >15.0){ // if timeout 
                    RCLCPP_INFO(this->get_logger(), "timed out");
                    robot_attempts ++;
                    
                    sfc = 6110; // go back and resend the robot cmd
                    if(robot_attempts == 5){
                       // sfc = 6030;
                       RCLCPP_INFO(this->get_logger(), "timed out, robot can't go to position");

                    }
                    
                }

                break;
            
            case 6130: // move robot to drop of aruco box pose
                robot_msg.cmd = 6;
                robot_msg.pose = {0.56395,0.0,0.2027,0.0,0.0,0.0}; 
                pub_robot_->publish(robot_msg);
                m_lastTime1 = m_clock->now().seconds(); // start timer for timeout

                sfc = 6140;

                break;
            
            case 6140:
                if(robot_status == 1){ // check if robot is done
                    robot_status = 0;
                    robot_attempts = 0;
                    sfc = 6150;
                }
                m_lastTime2 = m_clock->now().seconds(); // get time now
                if((m_lastTime2-m_lastTime1) >15.0){ // if timeout 
                    RCLCPP_INFO(this->get_logger(), "timed out");
                    robot_attempts ++;
                    
                    sfc = 6130; // go back and resend the robot cmd
                    if(robot_attempts == 5){
                       // sfc = 6030;
                       RCLCPP_INFO(this->get_logger(), "timed out, robot can't go to position");

                    }
                    
                }

                break;

            case 6150: // open the gripper 
                robot_msg.cmd = 20; // set gripper off
                robot_msg.pose = {-0.25,-0.25}; // pos do not matter
                pub_robot_->publish(robot_msg); // send to robot to set gripper off

                m_lastTime1 = m_clock->now().seconds(); // start timer for timeout
                sfc = 6160;
                break;
            
            case 6160: // waitting for gripper to open

                if(robot_status == 1){// Wait for gripper to be open
                    robot_status = 0;
                    robot_attempts = 0;
                    sfc = 6170;
                }
                m_lastTime2 = m_clock->now().seconds(); // get time now
                if((m_lastTime2-m_lastTime1) >7.0){ // if timeout 
                    RCLCPP_INFO(this->get_logger(), "timed out");
                    
                    sfc = 6150; // go back and resend the robot cmd
                    
                }

                break;
            
            case 6170: // move robot to sheach pose for golfball
                robot_msg.cmd = 6;
                robot_msg.pose = {0.34,0.0,0.168,0.0,1.45,0.0};
                pub_robot_->publish(robot_msg);
                m_lastTime1 = m_clock->now().seconds(); // start timer for timeout

                sfc = 6180;


                break;
            
            case 6180: // waitting for robot go to pose
                if(robot_status == 1){
                    robot_status = 0;
                    robot_attempts = 0;
                    sfc = 6190;
                }
                m_lastTime2 = m_clock->now().seconds(); // get time now
                if((m_lastTime2-m_lastTime1) >10.0){ // if timeout 
                    RCLCPP_INFO(this->get_logger(), "timed out");
                    
                    robot_attempts ++;
                    
                    
                    if(robot_attempts == 5){
                       // sfc = 6030;
                       RCLCPP_INFO(this->get_logger(), "timed out, robot can't go to position");

                    }
                    
                    sfc = 6170; // go back and resend the robot cmd
                    
                }


                break;
            
            case 6190:  // start camera to sharch for golfball
                ball_msg.data = true;
                pub_camera_ball_->publish(ball_msg); // setting the ball camera off
                status_ball = 0;

                m_lastTime1 = m_clock->now().seconds(); // start timer for timeout
                
                sfc = 6200;

                break;
            
            case 6200: // waitting for camera to find ball or timeout
                if(status_ball == 1)
                {
                    status_ball = 0;
                    sfc = 6210;
                }
                m_lastTime2 = m_clock->now().seconds(); // get time now
                if((m_lastTime2-m_lastTime1) >5.0){ // if timeout 
                    RCLCPP_INFO(this->get_logger(), "timed out, No golfball found");
                    
                    
                    
                }

                break;

            case 6210: // move robot to ball pose
                robot_msg.cmd = 16; // go to ball
                robot_msg.pose = {0.0,0.0,0.0,0.0,0.0,0.0};
                pub_robot_->publish(robot_msg);
                ball_msg.data = false;
                pub_camera_ball_->publish(aruco_msg);
                status_ball = 0;

                m_lastTime1 = m_clock->now().seconds();
                sfc = 6220;

                break;
            
            case 6220: // waitting for robot to go to pose
                if(robot_status == 1){
                    robot_status = 0;
                    robot_attempts = 0;
                    sfc = 6230;
                }
                m_lastTime2 = m_clock->now().seconds(); // get time now
                if((m_lastTime2-m_lastTime1) >10.0){ // if timeout 
                    RCLCPP_INFO(this->get_logger(), "timed out");
                    
                    sfc = 6190; // go back and resend the robot cmd
                    
                }
                break;
            
            case 6230: // close gripper
                robot_msg.cmd = 18; // close gripper
                robot_msg.pose = {0.37336,-0.007814,0.24958,0.0,1.57,0.0};
                pub_robot_->publish(robot_msg); // send to robot to set gripper close

                m_lastTime1 = m_clock->now().seconds(); // start timer for timeout

                sfc = 6240;

                break;
            
            case 6235: // waitting for gripper to close
                if(robot_status == 1){ // check if robot is done
                    robot_status = 0;
                    robot_attempts = 0;
                    sfc = 6240;
                }
                break;

            case 6240: // robot move to deapproch pose
                robot_msg.cmd = 6;
                robot_msg.pose = {0.34,0.0,0.13129,0.0,1.4,0.0};
                pub_robot_->publish(robot_msg);
                m_lastTime1 = m_clock->now().seconds(); // start timer for timeout

                sfc = 6250;
                break;

            case 6250: // waittting for robot to go to pose
                if(robot_status == 1){
                    robot_status = 0;
                    robot_attempts = 0;
                    sfc = 6260;
                }
                m_lastTime2 = m_clock->now().seconds(); // get time now
                if((m_lastTime2-m_lastTime1) >10.0){ // if timeout 
                    RCLCPP_INFO(this->get_logger(), "timed out");
                    
                    robot_attempts ++;
                    
                    
                    if(robot_attempts == 5){
                       // sfc = 6030;
                       RCLCPP_INFO(this->get_logger(), "timed out, robot can't go to position");

                    }
                    
                    sfc = 6240; // go back and resend the robot cmd
                    
                }
                break;
            
            case 6260: // make mobile base move to acuco code drop off point

                if(status_aruco == 5){ // aruco code green
                    pub_mobile_->publish(house_pose[0]);
                    sfc = 6270;
                }
                else if( status_aruco == 20){ // aruco code yellow
                    pub_mobile_->publish(house_pose[1]);
                    sfc = 6270;
                }
                else if( status_aruco == 6){ // aruco code red
                    pub_mobile_->publish(house_pose[2]);
                    sfc = 6270;
                }

                break;
            
            case 6270: // waitting for mobile robot to drive to point
                if(status_mobile == 1){
                    status_mobile = 0;
                    sfc = 6280;
                }

                break;
            
            case 6280: // move robot to the floor (to drop of ball)
                robot_msg.cmd = 6;
                robot_msg.pose = {0.28,0.0,0.0,0.0,1.57,0.0};
                pub_robot_->publish(robot_msg);
                m_lastTime1 = m_clock->now().seconds(); // start timer for timeout

                sfc = 6250;
                break;
            
            case 6290: // waitting for robot to go to pose
                if(robot_status == 1){
                    robot_status = 0;
                    robot_attempts = 0;
                    sfc = 6300;
                }
                m_lastTime2 = m_clock->now().seconds(); // get time now
                if((m_lastTime2-m_lastTime1) >10.0){ // if timeout 
                    RCLCPP_INFO(this->get_logger(), "timed out");
                    
                    robot_attempts ++;
                    
                    
                    if(robot_attempts == 5){
                       // sfc = 6030;
                       RCLCPP_INFO(this->get_logger(), "timed out, robot can't go to position");

                    }
                    
                    sfc = 6280; // go back and resend the robot cmd
                    
                }
                break;
            
            case 6300: // open gripper
                robot_msg.cmd = 19; // set gripper off
                robot_msg.pose = {0.37336,-0.007814,0.24958,0.0,1.57,0.0}; // pos do not matter
                pub_robot_->publish(robot_msg); // send to robot to set gripper off

                m_lastTime1 = m_clock->now().seconds(); // start timer for timeout
                sfc = 6310;

                break;
            
            case 6310: // waitting for gripper to open
                 if(robot_status == 1){ // check if robot is done
                    robot_status = 0;
                    robot_attempts = 0;
                    sfc = 6240;
                }

                break;
            
            case 6320: // move robot to shearch pose
                robot_msg.cmd = 6;
                robot_msg.pose = {0.34,0.0,0.13129,0.0,1.4,0.0};
                pub_robot_->publish(robot_msg);
                m_lastTime1 = m_clock->now().seconds(); // start timer for timeout

                sfc = 6330;
                break;
            
            case 6330: // waitiing for robot to go to pose
                if(robot_status == 1){
                    robot_status = 0;
                    robot_attempts = 0;
                    sfc = 6335;
                }
                m_lastTime2 = m_clock->now().seconds(); // get time now
                if((m_lastTime2-m_lastTime1) >10.0){ // if timeout 
                    RCLCPP_INFO(this->get_logger(), "timed out");
                    
                    robot_attempts ++;
                    
                    
                    if(robot_attempts == 5){
                       // sfc = 6030;
                       RCLCPP_INFO(this->get_logger(), "timed out, robot can't go to position");

                    }
                    
                    sfc = 6320; // go back and resend the robot cmd
                    
                }
                break;
            
            case 6335: // add one to packages count
                package_count ++;
                break;

            case 6340: // make mobile robot to go back to next pose at "pakkeleveringen"
                
                if(package_count == 3){
                    sfc = 6999;
                }
                else{
                    pub_mobile_->publish(trolly_pose[package_count]);
                    sfc = 6350;
                }
                break;
             
            case 6350: // waiiting for mobile robot to go to pose
                if(status_mobile == 1){
                    status_mobile = 0;
                    sfc = 6360;
                }
                break;
            
            case 6360: // repeat for next package
                sfc = 6010;
                break;
            
            
            
            case 6999: // tasl 13 done

                 break;

            //////////////////END TASK 13 from case 6000-6999 ///////////////////////////////////////////////////////////////////////////////////
            /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
            /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

            default:
                break;
            }
            RCLCPP_INFO(this->get_logger(), "sfc: %d",sfc);
        }

        bool service_done_ = false;

        bool is_service_done() const {return this->service_done_;}


        // crust arm service
        rclcpp::Client<crust_msgs::srv::RobotCmdSrv>::SharedPtr robot_client_ ;

        // vaccum
        rclcpp::Subscription<std_msgs::msg::Int8>::SharedPtr sub_vac_;
        rclcpp::Publisher<std_msgs::msg::Int8>::SharedPtr pub_vac_;

        // crust arm
        rclcpp::Publisher<crust_msgs::msg::RobotCmdMsg>::SharedPtr pub_robot_;
        rclcpp::Subscription<std_msgs::msg::Int8>::SharedPtr sub_robot_;

        // callback group
        rclcpp::CallbackGroup::SharedPtr callback_group_subscriber1_;
        rclcpp::CallbackGroup::SharedPtr callback_group_subscriber2_;

        //camera 
        rclcpp::Subscription<std_msgs::msg::Int8>::SharedPtr sub_camera_aruco_;
        rclcpp::Subscription<std_msgs::msg::Int8>::SharedPtr sub_camera_ball_;
        rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr pub_camera_aruco_;
        rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr pub_camera_ball_;
        int32_t status_aruco = 0;
        int32_t status_ball = 0;
        std_msgs::msg::Bool aruco_msg;
        std_msgs::msg::Bool ball_msg;


        rclcpp::TimerBase::SharedPtr timer_;
        size_t count_;
        int32_t sfc = 0;

        clock_t t_time1;
        clock_t t_time2;
        float tt;
        std_msgs::msg::Int8 vac_msg;
        
        rclcpp::Clock::SharedPtr m_clock;
        double m_lastTime1;
        rclcpp::TimerBase::SharedPtr m_timer1;
        double m_lastTime2;
        rclcpp::TimerBase::SharedPtr m_timer2;
        int8_t robot_status = 0;
        crust_msgs::msg::RobotCmdMsg robot_msg;
        int8_t robot_attempts = 0;
        int8_t package_count = 0;


        // mobile robot movement abs topic: /goal_pose
        geometry_msgs::msg::PoseStamped target_pose;
        geometry_msgs::msg::TransformStamped transform_pose;
        std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
        std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
        rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pub_mobile_;
        rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr sub_mobile_;
        
        int32_t status_mobile = 0;

        /// the 3 packages trolly pose, make as vector
        geometry_msgs::msg::PoseStamped trolly_pose[3]; 
        /// the 3 house pose, make as vector
        geometry_msgs::msg::PoseStamped house_pose[3]; // red 0 // yelllow 1 // green 2

};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::executors::MultiThreadedExecutor executor;
  auto main_program = std::make_shared<MainProgram>();
  executor.add_node(main_program);
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Server Ready.");
  executor.spin();
  
  
  rclcpp::shutdown();
}