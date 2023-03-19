

#include "rclcpp/rclcpp.hpp"
#include <memory>

#include <stdio.h>
#include <chrono>
#include <functional>
#include <cstdlib>
#include <future>

//#include "std_msgs/int8.hpp"
#include <std_msgs/msg/int8.hpp>
#include <thread>
#include <string>
#include "std_msgs/msg/string.hpp"
#include "crust_msgs/srv/robot_cmd_srv.hpp"
#include "crust_msgs/msg/robot_cmd_msg.hpp"
#include <time.h>

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
            //MainProgram::main_program();
        }
    
    private:
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
            
            switch (sfc)
            {
            case 0:
                // pack robot down
                RCLCPP_INFO(this->get_logger(), "start timer");
                t_time1 = clock();
                m_lastTime1 = m_clock->now().seconds();
                
                sfc = 10;

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
                    sfc = 90;
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
                RCLCPP_INFO(this->get_logger(), "robot_16");
                robot_msg.cmd = 16;
                robot_msg.pose = {0.0,0.0,0.0,0.0,0.0,0.0};
                pub_robot_->publish(robot_msg);
                t_time1 = clock();
                sfc = 120;
                break;
            
            case 120:
                if(robot_status == 1){
                    robot_status = 0;
                    sfc = 130;
                }
                if((float)((t_time2 -t_time1)*10.0/CLOCKS_PER_SEC) >5.0){
                    sfc = 140;
                }
                break;
            
            case 130:
                RCLCPP_INFO(this->get_logger(), "done");
                sfc = 140;
                break;
            
            case 140:
                RCLCPP_INFO(this->get_logger(), "timed out, restarting program");
                sfc = 0;
                break;

            default:
                break;
            }
            RCLCPP_INFO(this->get_logger(), "sfc: %d",sfc);
        }

        bool service_done_ = false;

        bool is_service_done() const {return this->service_done_;}
        rclcpp::Client<crust_msgs::srv::RobotCmdSrv>::SharedPtr robot_client_ ;
        rclcpp::Subscription<std_msgs::msg::Int8>::SharedPtr sub_vac_;
        rclcpp::Publisher<std_msgs::msg::Int8>::SharedPtr pub_vac_;
        rclcpp::Publisher<crust_msgs::msg::RobotCmdMsg>::SharedPtr pub_robot_;
        rclcpp::Subscription<std_msgs::msg::Int8>::SharedPtr sub_robot_;
        rclcpp::CallbackGroup::SharedPtr callback_group_subscriber1_;
        rclcpp::CallbackGroup::SharedPtr callback_group_subscriber2_;
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