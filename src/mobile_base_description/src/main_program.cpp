

#include "rclcpp/rclcpp.hpp"
#include <memory>

#include <stdio.h>
#include <chrono>
#include <functional>
#include <cstdlib>
#include <future>

//#include "std_msgs/int8.hpp"
#include <std_msgs/msg/int8.hpp>
#include <std_msgs/msg/bool.hpp>
#include <thread>
#include <string>
#include "std_msgs/msg/string.hpp"
#include "crust_msgs/srv/robot_cmd_srv.hpp"
#include "crust_msgs/msg/robot_cmd_msg.hpp"
#include <time.h>
#include <geometry_msgs/msg/pose_stamped.hpp>
using std::placeholders::_1;
using std::placeholders::_2;


using namespace std::chrono_literals;

class MainProgram : public rclcpp::Node
{
    public:
        MainProgram() : Node("MainProgram")
        {   
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
            //MainProgram::main_program();
        }
    
    private:

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
                
                sfc = 155;

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

            
            case 155:
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


        // mobile robot movement
        geometry_msgs::msg::PoseStamped mobile_pose;
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