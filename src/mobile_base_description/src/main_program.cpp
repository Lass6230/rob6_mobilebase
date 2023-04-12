

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
#include <sensor_msgs/msg/joy.hpp>


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
            
            timer_ = this->create_wall_timer(1000ms, std::bind(&MainProgram::timer_callback, this),callback_group_subscriber2_ );
            //sub_vac_ = this->create_subscription<std_msgs::msg::Int8>("vaccumControl", 10, std::bind(&MainProgram::vacCallback, this, _1), sub1_opt);
            pub_vac_ = this->create_publisher<std_msgs::msg::Int8>("vaccumControl",10);

            pub_robot_ = this->create_publisher<crust_msgs::msg::RobotCmdMsg>("/robot_cmd_msg",10);
            sub_robot_ = this->create_subscription<std_msgs::msg::Int8>("/robot_cmd_status",10,std::bind(&MainProgram::subRobotStatus, this, _1), sub1_opt);
            
            // camera sub and pub
            sub_camera_aruco_ = this->create_subscription<std_msgs::msg::Int8>("/status_aruco",10,std::bind(&MainProgram::subArucoStatus, this, _1), sub1_opt);
            sub_camera_ball_ = this->create_subscription<std_msgs::msg::Int8>("/status_ball",10,std::bind(&MainProgram::subBallStatus, this, _1), sub1_opt);
            pub_camera_aruco_ = this->create_publisher<std_msgs::msg::Bool>("/search_aruco",10);
            pub_camera_ball_ = this->create_publisher<std_msgs::msg::Int8>("/search_golfball",10);

            pub_mobile_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/goal",10);
            pub_mobile_relative_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/goal_relative",10);
            sub_mobile_ = this->create_subscription<std_msgs::msg::Int32>("/navigation_feedback", 10,std::bind(&MainProgram::subMobileStatus, this, _1), sub1_opt);
            
            sub_linefollow_ = this->create_subscription<std_msgs::msg::Int32>("/lineStatus", 10,std::bind(&MainProgram::subLineStatus, this, _1), sub1_opt);
            pub_linefollow_ = this->create_publisher<std_msgs::msg::Int32>("/cmd_lineFollow", 10);

            sub_joy_status_ = this->create_subscription<std_msgs::msg::Bool>("/joy_status",10,std::bind(&MainProgram::subJoyStatus, this, _1), sub1_opt);
            //sub_joy_ = this->create_subscription<sensor_msgs::msg::Joy>("/joy",10,std::bind(&MainProgram::subJoyStatus, this, _1), sub1_opt);
            
            ///////////// Line status og cmd koder ////////////////////////////

                //STATUS MED KOMMANDO NØDVENDIG
                // 20: robotten har fundet en sammenfletning og venter på en kommando
                // 30: Robotten har fundet et skarpt sving og venter på en komando

                //STATUS UDEN KOMMANDO NØDVENDIG
                // 90: Robotten har mistet linjen
                
                // Robottens status kode når den bare kører uden problemer
                // de forskellige parameter bliver lagt sammen til en enkel kode alt efter hvad er aktiveret
                // hvis linemode er aktiveret så vil statuskoden være et ulige tal
                //
                // 1: linefollowing aktiveret
                // 10: forsæt fremad efter mistet linje aktiveret
                // 12: Skarp sving aktiveret
                // 14: Sammenfletning aktiveret
                // 2: Kør til højre i sammenfeltning aktiveret
                // 90: tabt linen

                // Eksempel Kode:
                // linefollowing aktiveret med sammenfletning og kør til højre aktiveret
                // kode: 17
                
                // linefollowing aktiveret med sammenfletning og kør til venstre
                // kode: 15

                // linefollowing deaktiveret men sammenfletning og kør til venstre aktiveret 
                // kode: 14



                //CMD
                // 0: Slå linefollowing fra
                // 1: Aktiver linefollowing uden nogen skarpt sving og sammenfletning
                // 2: Aktiver linefollowing hvor robotten fortsætter fremad efter den mister linjen istedet for at finde linjen igen
                // 3: Aktiver linefollowing hvor robotten i en sammenfletning kører til højre
                // 4: Aktiver linefollowing hvor robotten laver et skarpt sving
                // 5: Aktiver linefollowing hvor robotten i en sammenfletning kører til venstre
                // 6: Sæt robotten igang når den venter på kommando


             ///////////// Line status og cmd koder ////////////////////////////
            

            // INIT poseStamped values for
            tf2::Quaternion quat_trolly[3];

            //gobal pose for the wall diretly outside group room in maps/0_04res/map.pgm
            quat_trolly[0].setRPY(0.0, 0.0, 0.0);
            trolly_pose[0].pose.position.x = 4.812;
            trolly_pose[0].pose.position.y = -3.79;
            trolly_pose[0].pose.orientation.z = -0.8026999780103126;
            trolly_pose[0].pose.orientation.w = 0.5963830524941531; //tf2::toMsg(quat_trolly[0]);

            //gobal pose for the wall diretly outside group room in maps/0_04res/map.pgm




            quat_trolly[1].setRPY(0.0, 0.0, 0.0);
            trolly_pose[1].pose.position.x = 4.728928565979004;
            trolly_pose[1].pose.position.y = -3.7367677688598633;
            //trolly_pose[1].pose.orientation = tf2::toMsg(quat_trolly[1]);
            trolly_pose[1].pose.orientation.z = -0.8026999780103126;
            trolly_pose[1].pose.orientation.w = 0.5963830524941531;


            quat_trolly[2].setRPY(0.0, 0.0, 0.0);
            trolly_pose[2].pose.position.x = 4.637298107147217;
            trolly_pose[2].pose.position.y = -3.852802276611328;
            //trolly_pose[2].pose.orientation = tf2::toMsg(quat_trolly[2]);
            trolly_pose[2].pose.orientation.z = -0.7844494034227334;
            trolly_pose[2].pose.orientation.w = 0.6201928195889709;

            
            tf2::Quaternion quat_house[3];

            quat_house[2].setRPY(0.0, 0.0, 0.0); // grøn
            house_pose[2].pose.position.x = 5.874170780181885;
            house_pose[2].pose.position.y = -4.7942585945129395;
            //house_pose[0].pose.orientation = tf2::toMsg(quat_house[0]);
            house_pose[2].pose.orientation.z = -0.8087978459540437;
            house_pose[2].pose.orientation.w = 0.5880867660304038;



            quat_house[1].setRPY(0.0, 0.0, 0.0); //gul
            house_pose[1].pose.position.x = 6.1005940437316895;
            house_pose[1].pose.position.y = -4.889570236206055;
            //house_pose[1].pose.orientation = tf2::toMsg(quat_house[1]);
            house_pose[1].pose.orientation.z = -0.798157318060768;
            house_pose[1].pose.orientation.w = 0.6024490813554636;





            quat_house[0].setRPY(0.0, 0.0, 0.0); //rød
            house_pose[0].pose.position.x = 6.2147216796875;
            house_pose[0].pose.position.y = -4.94196891784668;
            //house_pose[2].pose.orientation = tf2::toMsg(quat_house[2]);
            house_pose[0].pose.orientation.z = -0.8032736794291202;
            house_pose[0].pose.orientation.w = 0.5956101039576168;





            // init task_10_pose
            tf2::Quaternion quat_task10;
            quat_task10.setRPY(0.0, 0.0, 0.0);
            task_10_pose.pose.position.x = 4.693809986114502;
            task_10_pose.pose.position.y = -5.980169296264648;
            //task_10_pose.pose.orientation = tf2::toMsg(quat_task10);
            task_10_pose.pose.orientation.z = -0.13493156158721403;
            task_10_pose.pose.orientation.w = 0.9908549206052498;

            // init task_4_pose
            tf2::Quaternion quat_task4;
            quat_task4.setRPY(0.0, 0.0, 0.0);
            task_4_pose.pose.position.x = 3.079353094100952;
            task_4_pose.pose.position.y = -0.5278006196022034;
            //task_4_pose.pose.orientation = tf2::toMsg(quat_task4);
            task_4_pose.pose.orientation.z = 0.9955758379458963;
            task_4_pose.pose.orientation.w = 0.09396143303678546;

            // init golf_hole_pose
            tf2::Quaternion quat_golf_hole;
            quat_task4.setRPY(0.0, 0.0, 0.0);
            golf_hole_pose.pose.position.x = 6.758194923400879;
            golf_hole_pose.pose.position.y = -4.097546100616455;
            //golf_hole_pose.pose.orientation = tf2::toMsg(quat_golf_hole);
            golf_hole_pose.pose.orientation.z = 0.6340538789350814;
            golf_hole_pose.pose.orientation.w = 0.7732888713846703;

            // init golfball_sheach_pose
            tf2::Quaternion quat_golfball_sheach;
            quat_task4.setRPY(0.0, 0.0, 0.0);
            golfball_sheach_pose.pose.position.x = 7.370445251464844;
            golfball_sheach_pose.pose.position.y = -3.407285690307617;
            //golfball_sheach_pose.pose.orientation = tf2::toMsg(quat_golfball_sheach);
            golfball_sheach_pose.pose.orientation.z = 0.5665723056213117;
            golfball_sheach_pose.pose.orientation.w = 0.8240120281300213;

            header:
  stamp:
    sec: 1681259780
    nanosec: 493738628
  frame_id: map
pose:
  position:
    x: 7.370445251464844
    y: -3.407285690307617
    z: 0.0
  orientation:
    x: 0.0
    y: 0.0
    z: 0.5665723056213117
    w: 0.8240120281300213
---




            // golfball release
            golfball_release_pose.pose.position.x = 5.028180122375488;
            golfball_release_pose.pose.position.y = -3.191586971282959;
            //golfball_sheach_pose.pose.orientation = tf2::toMsg(quat_golfball_sheach);
            golfball_release_pose.pose.orientation.z = -0.17158369066287102;
            golfball_release_pose.pose.orientation.w = 0.9851695473868994;




            speed.pose.position.x = 6.314323425292969;
            speed.pose.position.y = -7.268889427185059;
            //golfball_sheach_pose.pose.orientation = tf2::toMsg(quat_golfball_sheach);
            speed.pose.orientation.z = -0.7979435037636673;
            speed.pose.orientation.w = 0.6027322496775516;

           
        }
    
    private:
        void subJoyStatus(const std_msgs::msg::Bool & msg){
            joy_status = msg.data;

        }
        void subLineStatus(const std_msgs::msg::Int32 & msg){
            status_linefollow = msg.data;
        }

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
            if(joy_status == true)
            {

            switch (sfc)
            {
            case 0:
               
                sfc = 10000;//6000;//4000;//155;//1100;

                break;
            
            //////////////////BEGINING Start first gate (3) from case 2000-2999 /////////////////////////////////////////////////////
            ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
            ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

            case 2000:
                sfc = 2009;
                break;

            case 2009:
                linefollow_msg.data = 0;
                pub_linefollow_ -> publish(linefollow_msg);
                aruco_msg.data = false; 
                pub_camera_aruco_->publish(aruco_msg); // setting the aruco camera off
                ball_msg.data = 0;
                pub_camera_ball_->publish(ball_msg); // setting the ball camera off
                status_aruco = 0;
                status_ball = 0;

                sfc=2010;
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
                RCLCPP_INFO(this->get_logger(), "send command to pack down robot");
                robot_msg.cmd = 3;//6;//3;
                robot_msg.pose = {0.0,0.0,0.0,0.0,0.0,0.0};//{0.29200,0.0,-0.12,0.0,1.45,0.0};//{0.0,0.0,0.0,0.0,0.0,0.0};
                pub_robot_->publish(robot_msg);
                m_lastTime1 = m_clock->now().seconds();
                sfc = 2040;
                break;
            
            case 2040: // wait for robot to be packed down
                if(robot_status == 1){ // check if robot is done
                    robot_status = 0;
                    robot_attempts = 0;
                    sfc = 2050;
                }
                m_lastTime2 = m_clock->now().seconds(); // get time now
                if((m_lastTime2-m_lastTime1) >25.0){ // if timeout 
                    RCLCPP_INFO(this->get_logger(), "timed out");
                    robot_attempts ++;
                    
                    sfc = 2030; // go back and resend the robot cmd
                    if(robot_attempts == 5){
                       
                       RCLCPP_INFO(this->get_logger(), "timed out, robot can't go to position");

                    }
                    
                }
                break;
            
            case 2050: // start line follower
                RCLCPP_INFO(this->get_logger(), "start line follower");
                linefollow_msg.data = 1;
                pub_linefollow_->publish(linefollow_msg);
                m_lastTime1 = m_clock->now().seconds();
                sfc= 2060;
                break;
            
            case 2060: // waitting for line follower to be done
                if (status_linefollow == 20){ //line following har fundet en sammenfletning eller skarpt sving og venter på en svar fra ros
                    status_linefollow = 0;
                    sfc = 2061;  // Ændres tilbage til 2070 
                }
                m_lastTime2 = m_clock->now().seconds();
                if ((m_lastTime2-m_lastTime1)> 45.0){
                    RCLCPP_INFO(this->get_logger(),"timed out");
                }
                break;
            
            case 2061: // stop line follower
                RCLCPP_INFO(this->get_logger(), "stop line follow");
                linefollow_msg.data = 0;
                pub_linefollow_->publish(linefollow_msg);
                //m_lastTime1 = m_clock->now().seconds();
                sfc= 2062;
                break;
            

            case 2062:
                RCLCPP_INFO(this->get_logger(), "nav one meter");
                target_pose.pose.position.x = 0.5;//update mig!!!
                target_pose.pose.position.y = 0.0;
                pub_mobile_relative_->publish(target_pose);
                sfc = 2063;
                break;
            
            case 2063:
                if(status_mobile == 1){
                    status_mobile = 0;
                    sfc = 9000;
                    //sfc = 2064;
                }
                break;

            // case 2062:
            //     linefollow_msg.data = 10;
            //     pub_linefollow_->publish(linefollow_msg);
            //     linefollow_msg.data = 6;
            //     sfc = 20621;
            //     break;

            // case 20621:
            //     pub_linefollow_->publish(linefollow_msg);
            //     m_lastTime1 = m_clock->now().seconds();
            //     sfc= 2063;
            //     break;
            
            // case 2063: // waitting for line follower to be done
            //     if (status_linefollow == 60){ //line following har fundet en sammenfletning eller skarpt sving og venter på en svar fra ros
            //         status_linefollow = 0;
            //         sfc = 2064;  // Ændres tilbage til 2070 
            //     }
            //     m_lastTime2 = m_clock->now().seconds();
            //     if ((m_lastTime2-m_lastTime1)> 45.0){
            //         RCLCPP_INFO(this->get_logger(),"timed out");
            //     }
            //     break;

            case 2064:
                RCLCPP_INFO(this->get_logger(), "nav golf release");
                pub_mobile_->publish(golfball_release_pose);
                sfc = 2065;

                break;
            
            case 2065:
                if(status_mobile == 1){
                    status_mobile = 0;
                    sfc = 2070;
                }
                break;

            
            case 2070: // set robot down in front of the mobile base ready for the ball releaser
                RCLCPP_INFO(this->get_logger(), "arm to release");
                robot_msg.cmd = 6;
                robot_msg.pose = {0.27,0.13,-0.12,0.0,1.57,0.48};
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
                //linefollow_msg.data = 1; // set linefollowing til og kør til venstre i sammenfletning
                RCLCPP_INFO(this->get_logger(), "start line follow 11");
                linefollow_msg.data = 11; //start line follow igen, led mod højre
                pub_linefollow_->publish(linefollow_msg);
                sfc = 2100;

                break;
            
            // not used
            case 2095:
                linefollow_msg.data = 6;
                pub_linefollow_->publish(linefollow_msg);
                sfc = 2096;
                break;
            // not used
            case 2096:
                if (status_linefollow == 15){
                    status_linefollow = 0;
                    sfc = 2099;
                }
                break;
            // not used
            case 2099: // start line following igen vi vil gerne have den til at køre til venstre i sammenfletningen
                linefollow_msg.data = 5; // set linefollowing til og kør til venstre i sammenfletning
                pub_linefollow_->publish(linefollow_msg);
                sfc = 2100;

                break;

            case 2100:
                if (status_linefollow == 20) { //se hvornår linefollow rammer sammenfletning / skarp sving
                    status_linefollow = 0;
                    sfc = 2110;
                }

                break;
            
            case 2110:
                RCLCPP_INFO(this->get_logger(), "stop line follow");
                linefollow_msg.data = 0;
                pub_linefollow_->publish(linefollow_msg);
                sfc = 2111;

                break;

            case 2111: // send command to pack down robot
                RCLCPP_INFO(this->get_logger(), "arm to 0,0,0");
                robot_msg.cmd = 3;//6;//3;
                robot_msg.pose = {0.0,0.0,0.0,0.0,0.0,0.0};//{0.29200,0.0,-0.12,0.0,1.45,0.0};//{0.0,0.0,0.0,0.0,0.0,0.0};
                pub_robot_->publish(robot_msg);
                m_lastTime1 = m_clock->now().seconds();
                sfc = 2112;
                break;
            
            case 2112: // wait for robot to be packed down
                if(robot_status == 1){ // check if robot is done
                    robot_status = 0;
                    robot_attempts = 0;
                    sfc = 2113;
                }
                m_lastTime2 = m_clock->now().seconds(); // get time now
                if((m_lastTime2-m_lastTime1) >25.0){ // if timeout 
                    RCLCPP_INFO(this->get_logger(), "timed out");
                    robot_attempts ++;
                    
                    sfc = 2111; // go back and resend the robot cmd
                    if(robot_attempts == 5){
                       
                       RCLCPP_INFO(this->get_logger(), "timed out, robot can't go to position");

                    }
                    
                }
                break;
            
            
            case 2113:
                //linefollow_msg.data = 7;
                //pub_linefollow_->publish(linefollow_msg);
                RCLCPP_INFO(this->get_logger(), "nav to search pose");
                pub_mobile_->publish(golfball_sheach_pose);
                sfc = 2115;
                break;
            case 2115:
                //linefollow_msg.data = 6;
                //pub_linefollow_->publish(linefollow_msg);
                if(status_mobile == 1){
                    status_mobile = 0;
                    sfc = 2999;
                }
                //sfc = 2116;
                break;
            
            case 2116:
                if (status_linefollow == 13){
                    status_linefollow = 0;
                    sfc = 2117;
                }
                break;
            case 2117:
                linefollow_msg.data = 4;
                pub_linefollow_->publish(linefollow_msg);
                sfc = 2120;
                break;
            case 2120:
                if (status_linefollow == 20){ // når vi ser det næste cross burde vi være ved opgaven
                    status_linefollow = 0;
                    sfc = 2130; 
                }

                break;
            
            case 2130:
                linefollow_msg.data = 0; // sluk for line following så vi kan begynde at bruge navigation
                pub_linefollow_->publish(linefollow_msg);
                
                sfc = 2999;
                break;
            
            case 2140:      
                // Indsæt muligvis et punkt som er bedre egnet til at se all golf bolde før vi stater med at løse Task 12

                break;
            
            case 2150:

                break;
            
            case 2999:
                sfc = 4000;
                break;

            //////////////////END Start first gate (3) from case 2000-2999 /////////////////////////////////////////////////////
            ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
            ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////




            //////////////////BEGINING TASK 12 from case 4000-4999 /////////////////////////////////////////////////////////////////
            ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
            ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

            case 4000:
                sfc = 4010;
                break;

            case 4010: // open gripper
                RCLCPP_INFO(this->get_logger(), "open gripper, camera off");
                robot_msg.cmd = 19; // set gripper off and setting the cameras off
                robot_msg.pose = {0.37336,-0.007814,0.24958,0.0,1.57,0.0}; // pos do not matter
                pub_robot_->publish(robot_msg); // send to robot to set gripper off

                aruco_msg.data = false; 
                pub_camera_aruco_->publish(aruco_msg); // setting the aruco camera off
                ball_msg.data = 0;
                pub_camera_ball_->publish(ball_msg); // setting the ball camera off
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
                if((m_lastTime2-m_lastTime1) >10.0){ // if timeout 
                    RCLCPP_INFO(this->get_logger(), "timed out");
                    
                    sfc = 4010; // go back and resend the robot cmd
                    //
                    
                }
                break;

            case 4030: // set pose for robot søg ned lige foran
                RCLCPP_INFO(this->get_logger(), "look in front");
                robot_msg.cmd = 6;
                robot_msg.pose = {0.34,0.0,0.168,0.0,1.57,0.0};
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
                RCLCPP_INFO(this->get_logger(), "looking for ball");
                ball_msg.data = 2;
                pub_camera_ball_->publish(ball_msg); // setting the ball camera on to orange and pink
                status_ball = 0;

                m_lastTime1 = m_clock->now().seconds(); // start timer for timeout
                
                sfc = 4060;
                break;

            case 4060: // waiiting for ball  IF ball jump to 4600
                if(status_ball == 1)
                {
                    RCLCPP_INFO(this->get_logger(), "ball found");
                    status_ball = 0;
                    sfc = 4600;
                }
                m_lastTime2 = m_clock->now().seconds(); // get time now
                if((m_lastTime2-m_lastTime1) >5.0){ // if timeout 
                    RCLCPP_INFO(this->get_logger(), "timed out, No golfball found");
                    ball_msg.data = 0;
                    pub_camera_ball_->publish(ball_msg); // setting the ball camera off
                   
                    status_ball = 0;
                    
                    sfc = 4070;
                    
                }

                break;
            
            case 4070: // send to robot to go to pose right looking down
                RCLCPP_INFO(this->get_logger(), "looking right");
                robot_msg.cmd = 6;
                robot_msg.pose = {0.23319,-0.24479,0.15955,0.0,1.57,-0.785398};
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
                RCLCPP_INFO(this->get_logger(), "start search");
                ball_msg.data = 2;
                pub_camera_ball_->publish(ball_msg); // setting the ball camera on orange and pink
                status_ball = 0;
                
                m_lastTime1 = m_clock->now().seconds(); // start timer for timeout
                sfc = 4100;
                break;
            
            case 4100: // søgning       if found jump to 4600
                if(status_ball == 1)
                {
                    RCLCPP_INFO(this->get_logger(), "ball found");
                    status_ball = 0;
                    sfc = 4600;
                }
                m_lastTime2 = m_clock->now().seconds(); // get time now
                if((m_lastTime2-m_lastTime1) >5.0){ // if timeout 
                    RCLCPP_INFO(this->get_logger(), "timed out, No golfball found");
                    ball_msg.data = 0;
                    pub_camera_ball_->publish(ball_msg); // setting the ball camera off
                   
                    status_ball = 0;
                    
                    sfc = 4110;
                    
                }

                break;
            
            case 4110: // set robot pose to look down left
                RCLCPP_INFO(this->get_logger(), "looking left");
                robot_msg.cmd = 6;
                robot_msg.pose = {0.23319,0.24479,0.15955,0.0,1.57,0.785398};
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
                RCLCPP_INFO(this->get_logger(), "start search");
                ball_msg.data = 2;
                pub_camera_ball_->publish(ball_msg); // setting the ball camera on orange and pink
                status_ball = 0;

                m_lastTime1 = m_clock->now().seconds(); // start timer for timeout
                
                sfc = 4140;
                break;
            
            case 4140: // søgning
                if(status_ball == 1)
                {
                    RCLCPP_INFO(this->get_logger(), "ball found");
                    status_ball = 0;
                    sfc = 4600;
                }
                m_lastTime2 = m_clock->now().seconds(); // get time now
                if((m_lastTime2-m_lastTime1) >7.0){ // if timeout 
                    RCLCPP_INFO(this->get_logger(), "timed out, No golfball found");
                    ball_msg.data = 0;
                    pub_camera_ball_->publish(ball_msg); // setting the ball camera off
                   
                    status_ball = 0;
                    
                    sfc = 4150;
                    
                }
        
                break;
            
            case 4145:
                sfc = 4999;

                break;

            case 4150: // send command to robot to look long midt
                RCLCPP_INFO(this->get_logger(), "look long");
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
                ball_msg.data = 2;
                pub_camera_ball_->publish(ball_msg); // setting the ball camera on orange and pink
                status_ball = 0;

                m_lastTime1 = m_clock->now().seconds(); // start timer for timeout
                
                sfc = 4180;

                break;
            
            case 4180:
                if(status_ball == 1)
                {
                    RCLCPP_INFO(this->get_logger(), "ball found");
                    status_ball = 0;
                    sfc = 4300;
                }
                m_lastTime2 = m_clock->now().seconds(); // get time now
                if((m_lastTime2-m_lastTime1) >5.0){ // if timeout 
                    RCLCPP_INFO(this->get_logger(), "timed out, No golfball found");
                    ball_msg.data = 0;
                    pub_camera_ball_->publish(ball_msg); // setting the ball camera off
                   
                    status_ball = 0;
                    
                    sfc = 4190;
                    
                }

                break;

            case 4190:// robot go to look long right
                RCLCPP_INFO(this->get_logger(), "look long right");
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
                ball_msg.data = 2;
                pub_camera_ball_->publish(ball_msg); // setting the ball camera on orange and pink
                status_ball = 0;

                m_lastTime1 = m_clock->now().seconds(); // start timer for timeout
                
                sfc = 4230;
                break;
                
            case 4230: // søgning
                if(status_ball == 1)
                {
                    RCLCPP_INFO(this->get_logger(), "ball found");
                    status_ball = 0;
                    sfc = 4300;
                }
                m_lastTime2 = m_clock->now().seconds(); // get time now
                if((m_lastTime2-m_lastTime1) >5.0){ // if timeout 
                    RCLCPP_INFO(this->get_logger(), "timed out, No golfball found");
                    ball_msg.data = 0;
                    pub_camera_ball_->publish(ball_msg); // setting the ball camera off
                   
                    status_ball = 0;
                    
                    sfc = 4240;
                    
                }
                break;
            
            case 4240: // robot to go to pose look long left
                RCLCPP_INFO(this->get_logger(), "look long left");
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
                       
                       RCLCPP_INFO(this->get_logger(), "timed out, robot can't go to position");

                    }
                    
                }

                break;
            
            case 4260: // send command to søg
                ball_msg.data = 2;
                pub_camera_ball_->publish(ball_msg); // setting the ball camera on orange and pink
                status_ball = 0;

                m_lastTime1 = m_clock->now().seconds(); // start timer for timeout
                
                sfc = 4270;
                break;
            
            case 4270:
                if(status_ball == 1)
                {
                    RCLCPP_INFO(this->get_logger(), "ball found");
                    status_ball = 0;
                    sfc = 4300;
                }
                m_lastTime2 = m_clock->now().seconds(); // get time now
                if((m_lastTime2-m_lastTime1) >5.0){ // if timeout 
                    RCLCPP_INFO(this->get_logger(), "timed out, No golfball found");
                    ball_msg.data = 0;
                    pub_camera_ball_->publish(ball_msg); // setting the ball camera off
                   
                    status_ball = 0;
                    
                    sfc = 4260;
                    
                }

                break;
            
            case 4280:

                break;

            case 4290:

                break;
            
            case 4300: // ball found long look, send command to mobile to drive to ball
                RCLCPP_INFO(this->get_logger(), "nav to ball");
                transform_pose = tf_buffer_->lookupTransform(
                    "workspace_center", "ball",
                    tf2::TimePointZero);
                target_pose.pose.position.x = transform_pose.transform.translation.x;
                target_pose.pose.position.y = transform_pose.transform.translation.y;
                
                tf2::fromMsg(transform_pose.transform.rotation, quat_tf_lookup);
                
                matrix.setRotation(quat_tf_lookup);
                matrix.getRPY(rot_r_lookup, rot_p_lookup, rot_y_lookup);
                quat_tf_lookup.setRPY(0.0,0.0,rot_y_lookup);
                target_pose.pose.orientation = tf2::toMsg(quat_tf_lookup);
                pub_mobile_relative_->publish(target_pose);
                sfc = 4310;
            
                break;

            case 4310: // waitting for mobile base to drive to ball pose
                if(status_mobile == 1){
                    RCLCPP_INFO(this->get_logger(), "nav complete");
                    status_mobile = 0;
                    sfc = 4010; // send the back to the shearch function hopefully it will catch it when lokking down in midt
                }
                break;
            


            case 4400: // søg der hvor den er
                ball_msg.data = 2;
                pub_camera_ball_->publish(ball_msg); // setting the ball camera off
                status_ball = 0;

                m_lastTime1 = m_clock->now().seconds(); // start timer for timeout
                
                sfc = 4410;
                break;


            case 4410:
                if(status_ball == 1)
                {
                    RCLCPP_INFO(this->get_logger(), "ball found");
                    status_ball = 0;
                    sfc = 4600;
                }
                m_lastTime2 = m_clock->now().seconds(); // get time now
                if((m_lastTime2-m_lastTime1) >5.0){ // if timeout 
                    RCLCPP_INFO(this->get_logger(), "timed out, No golfball found");
                    ball_msg.data = 0;
                    pub_camera_ball_->publish(ball_msg); // setting the ball camera off
                   
                    status_ball = 0;
                    
                    sfc = 4400;
                    
                }
                break;



            case 4600: // ball found, send command to go to ball
                RCLCPP_INFO(this->get_logger(), "go to ball and close");
                robot_msg.cmd = 16; // go to ball
                robot_msg.pose = {0.0,0.0,0.0,0.0,0.0,0.0}; 
                pub_robot_->publish(robot_msg);
                ball_msg.data = false;
                pub_camera_ball_->publish(ball_msg);
                status_ball = 0;

                m_lastTime1 = m_clock->now().seconds();
                sfc = 4610;


                 break;

            case 4610:
                if(robot_status == 1){ // check if robot is done
                    robot_status = 0;
                    robot_attempts = 0;
                    sfc = 4620;
                }
                m_lastTime2 = m_clock->now().seconds(); // get time now
                if((m_lastTime2-m_lastTime1) >25.0){ // if timeout 
                    RCLCPP_INFO(this->get_logger(), "timed out");
                    robot_attempts ++;
                    
                    sfc = 4400; // go back and resend the robot cmd
                    if(robot_attempts == 5){
                       // sfc = 6030;
                       RCLCPP_INFO(this->get_logger(), "timed out, robot can't go to position");

                    }
                    
                }
                break;
            
            case 4620: // send command to close gripper (Not used)

                robot_msg.cmd = 18; // close gripper
                robot_msg.pose = {-0.3,-0.3};
                pub_robot_->publish(robot_msg); // send to robot to set gripper close

                m_lastTime1 = m_clock->now().seconds(); // start timer for timeout

                sfc = 4630;
                break;
            
            case 4630: // waitting for gripper to close
                if(robot_status == 1){ // check if robot is done
                    robot_status = 0;
                    robot_attempts = 0;
                    sfc = 4640;
                }
                m_lastTime2 = m_clock->now().seconds(); // get time now
                if((m_lastTime2-m_lastTime1) >15.0){ // if timeout 
                    RCLCPP_INFO(this->get_logger(), "timed out");
                    robot_attempts ++;
                    
                    sfc = 4620; // go back and resend the robot cmd
                    if(robot_attempts == 5){
                       // sfc = 6030;
                       RCLCPP_INFO(this->get_logger(), "timed out, robot can't go to position");

                    }
                    
                }

                break;

            case 4640: // send command for robot to go up agian
                RCLCPP_INFO(this->get_logger(), "arm up");
                robot_msg.cmd = 6;
                robot_msg.pose = {0.34,0.0,0.168,0.0,1.45,0.0};
                pub_robot_->publish(robot_msg);
                m_lastTime1 = m_clock->now().seconds(); // start timer for timeout

                sfc = 4650;
                break;
            
            case 4650: // waitting for robot to go to pose
                if(robot_status == 1){
                    robot_status = 0;
                    robot_attempts = 0;
                    sfc = 4660;   //4660 /// husk sæt rigitgt
                }
                m_lastTime2 = m_clock->now().seconds(); // get time now
                if((m_lastTime2-m_lastTime1) >10.0){ // if timeout 
                    RCLCPP_INFO(this->get_logger(), "timed out");
                    
                    robot_attempts ++;
                    
                    
                    if(robot_attempts == 5){
                       // sfc = 6030;
                       RCLCPP_INFO(this->get_logger(), "timed out, robot can't go to position");

                    }
                    
                    sfc = 4640; // go back and resend the robot cmd
                    
                }
                break;
            
            case 4660: // send command to go to golf hole
                RCLCPP_INFO(this->get_logger(), "nav golf hole pose");
                pub_mobile_->publish(golf_hole_pose);
                sfc = 4670;
                break;
            
            case 4670: // waitting for robot to go golf hole
                if(status_mobile == 1){
                    status_mobile = 0;
                    sfc = 4680;
                }
                break;

            case 4680: // send command for robot to move to to the floor
                RCLCPP_INFO(this->get_logger(), "arm down");
                robot_msg.cmd = 6;
                robot_msg.pose = {0.34,0.0,-0.08,0.0,1.30,0.0};
                pub_robot_->publish(robot_msg);
                m_lastTime1 = m_clock->now().seconds(); // start timer for timeout

                sfc = 4690;
                break;
            
            case 4690: // waitting for robot to go to pose
                if(robot_status == 1){
                    robot_status = 0;
                    robot_attempts = 0;
                    sfc = 4700;
                }
                m_lastTime2 = m_clock->now().seconds(); // get time now
                if((m_lastTime2-m_lastTime1) >10.0){ // if timeout 
                    RCLCPP_INFO(this->get_logger(), "timed out");
                    
                    robot_attempts ++;
                    
                    
                    if(robot_attempts == 5){
                       // sfc = 6030;
                       RCLCPP_INFO(this->get_logger(), "timed out, robot can't go to position");

                    }
                    
                    sfc = 4680; // go back and resend the robot cmd
                    
                }
                break;
            
            case 4700: // open gripper
                RCLCPP_INFO(this->get_logger(), "open gripper");
                robot_msg.cmd = 19; // set gripper off and setting the cameras off
                robot_msg.pose = {0.37336,-0.007814,0.24958,0.0,1.57,0.0}; // pos do not matter
                pub_robot_->publish(robot_msg); // send to robot to set gripper off
                m_lastTime1 = m_clock->now().seconds(); // start timer for timeout
                sfc = 4710;
                break;
            
            case 4710: // waitting for gripper to open
                if(robot_status == 1){// Wait for gripper to be open
                    robot_status = 0;
                    robot_attempts = 0;
                    sfc = 4720;
                }
                m_lastTime2 = m_clock->now().seconds(); // get time now
                if((m_lastTime2-m_lastTime1) >5.0){ // if timeout 
                    RCLCPP_INFO(this->get_logger(), "timed out");
                    
                    sfc = 4700; // go back and resend the robot cmd
                    
                }
                break;
            
            case 4720: // send command to robot to go up from golf hole
                RCLCPP_INFO(this->get_logger(), "arm up");
                robot_msg.cmd = 6;
                robot_msg.pose = {0.34,0.0,0.168,0.0,1.45,0.0};
                pub_robot_->publish(robot_msg);
                m_lastTime1 = m_clock->now().seconds(); // start timer for timeout

                sfc = 4730;
                break;
            
            case 4730: // waitting for robot to go up from golf hole
                if(robot_status == 1){
                    robot_status = 0;
                    robot_attempts = 0;
                    sfc = 4740;
                }
                m_lastTime2 = m_clock->now().seconds(); // get time now
                if((m_lastTime2-m_lastTime1) >15.0){ // if timeout 
                    RCLCPP_INFO(this->get_logger(), "timed out");
                    robot_attempts ++;
                    
                    sfc = 4720; // go back and resend the robot cmd
                    if(robot_attempts == 5){
                       // sfc = 6030;
                       RCLCPP_INFO(this->get_logger(), "timed out, robot can't go to position");

                    }
                    
                }
                break;

            case 4740:
                golfball_counter ++;
                sfc = 4750;
                break;
            
            case 4750:
                if(golfball_counter == 4)
                {
                    RCLCPP_INFO(this->get_logger(), "four balls");
                    sfc = 4999;
                }
                else
                {
                    RCLCPP_INFO(this->get_logger(), "get another ball");
                    sfc = 4760;
                }

                break;
            
            case 4760:
                pub_mobile_->publish(golfball_sheach_pose);
                sfc = 4770;

                break;

            case 4770:
                if(status_mobile == 1){
                    status_mobile = 0;
                    sfc = 4780;
                }

                break;
            
            case 4780:
                sfc = 4010;

                break;

            case 4999:
                sfc = 5000;
                break;

            //////////////////END TASK 12 from case 4000-4999 /////////////////////////////////////////////////////////////////
            ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
            ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////





            //////////////////BEGINING Drive to  13 from case 5000-5999 ////////////////////////////////////////////////////////////
            ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
            ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

            case 5000:
                sfc = 5010;
                break;
            
            case 5010:
                robot_msg.cmd = 6;
                robot_msg.pose = {0.34,0.0,0.168,0.0,1.45,0.0};
                pub_robot_->publish(robot_msg);
                m_lastTime1 = m_clock->now().seconds(); // start timer for timeout

                sfc = 5020;
                break;
            
            case 5020:
                if(robot_status == 1){// Wait for robot to got to pose
                    robot_status = 0;
                    robot_attempts = 0;
                    sfc = 5030;
                }
                m_lastTime2 = m_clock->now().seconds(); // get time now
                if((m_lastTime2-m_lastTime1) >5.0){ // if timeout 
                    RCLCPP_INFO(this->get_logger(), "timed out");
                    
                    sfc = 5010; // go back and resend the robot cmd
                    
                }
                break;
            
            case 5030: // send command to drive to trolly 1
                pub_mobile_->publish(trolly_pose[0]);
                sfc = 5040;

                break;
            
            case 5040: // waitting for driveing to trolly 1
                if(status_mobile == 1){
                    status_mobile = 0;
                    sfc = 5050;
                }
                break;
            
            case 5050:
                sfc = 5999;
                break;
            
            case 5999:
                sfc = 6000;
                break;


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
                pub_camera_ball_->publish(ball_msg); // setting the ball camera off
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
                if((m_lastTime2-m_lastTime1) >10.0){ // if timeout 
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
                if((m_lastTime2-m_lastTime1) >20.0){ // if timeout 
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
                m_lastTime2 = m_clock->now().seconds(); // get time now
                if(status_aruco == 5|| status_aruco == 6 || status_aruco == 20) // check if aruco code is found for rød or green or yellow
                {
                    // the color code is store in the variable status_aruco
                    sfc = 6070;
                }
               
                if((m_lastTime2-m_lastTime1) >7.0){ // check if timeout 
                    RCLCPP_INFO(this->get_logger(), "timed out, no aruco code found");
                    
                    //sfc = 6050;
                    sfc = 6332;
                    
                    
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
                    sfc = 6110;
                }
                m_lastTime2 = m_clock->now().seconds(); // get time now
                if((m_lastTime2-m_lastTime1) > 35.0){ // if timeout 
                    RCLCPP_INFO(this->get_logger(), "timed out");
                    robot_attempts ++;
                    
                    sfc = 6050; // go back and resend the robot cmd
                    if(robot_attempts == 5){
                       // sfc = 6030;
                       RCLCPP_INFO(this->get_logger(), "timed out, robot can't go to position");

                    }
                    
                }

                break;
            
            // Not used
            case 6090: // close gripper
                robot_msg.cmd = 18; // close gripper
                robot_msg.pose = {-0.3,-0.3};
                pub_robot_->publish(robot_msg); // send to robot to set gripper close

                m_lastTime1 = m_clock->now().seconds(); // start timer for timeout

                sfc = 6100;

                break;
            /// not used
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
                if((m_lastTime2-m_lastTime1) >15.0){ // if timeout 
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
                ball_msg.data = 1;
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
                    
                    //sfc = 6190;
                    sfc = 6332;
                    
                    
                }

                break;

            case 6210: // move robot to ball pose
                robot_msg.cmd = 22; // go to ball
                robot_msg.pose = {0.0,0.0,0.0,0.0,0.0,0.0};
                pub_robot_->publish(robot_msg);
                ball_msg.data = false;
                pub_camera_ball_->publish(ball_msg);
                status_ball = 0;

                m_lastTime1 = m_clock->now().seconds();
                sfc = 6220;

                break;
            
            case 6220: // waitting for robot to go to pose
                if(robot_status == 1){
                    robot_status = 0;
                    robot_attempts = 0;
                    sfc = 6240;
                }
                m_lastTime2 = m_clock->now().seconds(); // get time now
                if((m_lastTime2-m_lastTime1) >25.0){ // if timeout 
                    RCLCPP_INFO(this->get_logger(), "timed out");
                    
                    sfc = 6190; // go back and resend the robot cmd
                    
                }
                break;
            // not used
            case 6230: // close gripper
                robot_msg.cmd = 18; // close gripper
                robot_msg.pose = {0.37336,-0.007814,0.24958,0.0,1.57,0.0};
                pub_robot_->publish(robot_msg); // send to robot to set gripper close

                m_lastTime1 = m_clock->now().seconds(); // start timer for timeout

                sfc = 6240;

                break;
            // not used
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
                    RCLCPP_INFO(this->get_logger(), "go to green");
                    sfc = 6270;
                }
                else if( status_aruco == 20){ // aruco code yellow
                    RCLCPP_INFO(this->get_logger(), "go to yellow"); 
                    pub_mobile_->publish(house_pose[1]);
                    sfc = 6270;
                }       
                else if( status_aruco == 6){ // aruco code red
                    RCLCPP_INFO(this->get_logger(), "go to red");
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
                robot_msg.pose = {0.28,0.0,-0.1,0.0,1.57,0.0};
                pub_robot_->publish(robot_msg);
                m_lastTime1 = m_clock->now().seconds(); // start timer for timeout

                sfc = 6290;
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
                    sfc = 6320;
                }

                break;
            
            case 6320: // move robot to search pose
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
            
            case 6332: // aruco or ball not found
                pub_mobile_->publish(golfball_sheach_pose);
                sfc = 6333;
                break;
            
            case 6333:
                if(status_mobile == 1){
                    status_mobile = 0;
                    sfc = 6335;
                }

                break;
            
            case 6335: // add one to packages count
                package_count ++;
                sfc = 6340;
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
                sfc = 7000;

                 break;

            //////////////////END TASK 13 from case 6000-6999 ///////////////////////////////////////////////////////////////////////////////////
            /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
            /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


            //////////////////Drive to task 10 from case 7000-7999 ///////////////////////////////////////////////////////////////////////////////////
            /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
            /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

            case 7000:
                sfc = 7010;
                break;
            
            case 7010:
                //pack down arm
                robot_msg.cmd = 3;
                robot_msg.pose = {0.34,0.0,0.168,0.0,1.45,0.0};
                pub_robot_->publish(robot_msg);
                m_lastTime1 = m_clock->now().seconds(); // start timer for timeout

                sfc = 7020;
                break;
            
            case 7020:
                if(robot_status == 1){// Wait for gripper to be open
                    robot_status = 0;
                    robot_attempts = 0;
                    sfc = 7030;
                }
                m_lastTime2 = m_clock->now().seconds(); // get time now
                if((m_lastTime2-m_lastTime1) >5.0){ // if timeout 
                    RCLCPP_INFO(this->get_logger(), "timed out");
                    
                    sfc = 7010; // go back and resend the robot cmd
                }
                break;  

            case 7030:
                //oekse port
                pub_mobile_->publish(task_10_pose);
                sfc = 7040;
                break;
            
            case 7040:
                if(status_mobile == 1){
                    status_mobile = 0;
                    sfc = 7050;
                }

                break;

            case 7050:
                sfc = 7999;
                break;

            case 7060:

                break;
            
            case 7070:

                break;
            
            case 7080:

                break;
            
            case 7090:

                break;
            
            case 7100:

                break;
            
            




            case 7999:
                sfc = 8000;

                break;

            //////////////////END Drive to task 10 from case 7000-7999 ///////////////////////////////////////////////////////////////////////////////////
            /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
            /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////




            //////////////////BEGIN Task 10 from case 8000-8999 ///////////////////////////////////////////////////////////////////////////////////
            /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
            /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
            case 8000: // bruger vi lidaren til at se hvornår vi kan køre?
                sfc = 8010;
                break;
            
            case 8010: // start line follow 
                //LAV LIDAR STUFF!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!

                linefollow_msg.data = 2; // Start line following hvor den forsætter efter den mister linje
                pub_linefollow_->publish(linefollow_msg);
                sfc = 8020;
                break;
            
            case 8020: // wait for line follow to be done
                RCLCPP_INFO(this->get_logger(), "Publishing: '%i'", status_linefollow);
                if (status_linefollow == 90){ // vent på at robotten mister linjen
                status_linefollow = 0;
                sfc = 8040;

                }
                break;
            
            
            // case 8030:
            //     if (status_linefollow == 50){ // når robotten har mistet linjen vil vi gerne have den til at bare køre lige så stille frem for at finde linjen igen
            //         status_linefollow = 0;
            //         sfc = 8999;
            //     }
            //     RCLCPP_INFO(this->get_logger(), "Publishing: '%i'", status_linefollow);
            //     break;
            
            // // NOT USED
            case 8040:
                RCLCPP_INFO(this->get_logger(), "Publishing: '%i'", status_linefollow);
                if (status_linefollow == 50){
                    status_linefollow = 0;
                    sfc = 8999;
                }
                break;
             /// NOT USED   
         



            case 8999:
                sfc = 9000;
                break;
            //////////////////END Task 10 from case 8000-8999 ///////////////////////////////////////////////////////////////////////////////////
            /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
            /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


            //////////////////Begin  case 9000-9999 ///////////////////////////////////////////////////////////////////////////////////
            /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
            /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
            case 9000:
                sfc = 9010;
                break;

            case 9010: // ved ikke om armen er ned på dette tidspunkt har efterladt denne case i det tilfælde

                pub_mobile_->publish(speed);
                sfc = 9015;
                break;
            
            case 9015:
                if(status_mobile == 1){
                    status_mobile = 0;
                    sfc = 9020;
                }
                break;

            case 9020: // måske tilføje en måde at bestemme hvilken vej vi drejer i tilfælde af linjen er horizontal som den ville være i dette tilfælde
                linefollow_msg.data = 8; // her slår vi normal line mode til igen så den vil prøve at finde linjen igen 
                pub_linefollow_->publish(linefollow_msg);
                sfc = 9025;
                break;

            case 9025: // måske tilføje en måde at bestemme hvilken vej vi drejer i tilfælde af linjen er horizontal som den ville være i dette tilfælde
                linefollow_msg.data = 1; // her slår vi normal line mode til igen så den vil prøve at finde linjen igen 
                pub_linefollow_->publish(linefollow_msg);
                sfc = 9030;
                break;
            
            
            case 9030:
                if (status_linefollow == 90){
                    status_linefollow = 0;
                    m_lastTime1 = m_clock->now().seconds();
                    sfc = 9040;
                }
                break;
            
            case 9040: // her vil vi gerne finde ud af hvornår vi er færdige med at køre banen
                m_lastTime2 = m_clock->now().seconds(); // ideen er at robotten vil bruge lang tid på at finde linjen igen når den er færdig med 9
                if (status_linefollow == 90 && (m_lastTime2 - m_lastTime1) > 5.0 ){  // derfor vil jeg tjekke om den har mistet linjen og om der er gået mere end 5 sekunder
                    status_linefollow = 0;
                    sfc = 9050;
                }
                // hvis den finder linjen igen så skal den bare gå tilbage til case 9030, der er en mulighed for at vi bare ender i et evigt loop så sikker os at det her virker
                else if (status_linefollow == 1)
                {
                    status_linefollow = 0;
                    sfc = 9030;
                }
                
                break;
            
            case 9050:
                sfc = 9999;
                break;
            
            case 9060:
                // Hvis vi ikke kan finde linjen konsitent kan vi sætte et Nav Goal som skal hjælpe med at finde linjen. 
                break;
            
            case 9999:
                sfc = 10000;
                break;

            //////////////////END case 9000-9999 ///////////////////////////////////////////////////////////////////////////////////
            /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
            /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
            




            //////////////////Drive to 4,2,11 case 10000-10999 ///////////////////////////////////////////////////////////////////////////////////
            /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
            /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

            case 10000:
                sfc = 10500;

                break;
            
            case 10010:

                break;
            
            case 10020:

                break;
            
            case 10030:

                break;
            
            case 10040:

                break;
            
            case 10050:

                break;

            case 10500:   

                pub_mobile_->publish(task_4_pose); 
                sfc = 10510;

                break;
            case 10510: 
                if(status_mobile == 1){
                    status_mobile = 0;
                    sfc = 10999;
                }
                break;

            case 10999: 
                sfc = 11000;
                break;

            //////////////////END task 9 case 10000-10999 ///////////////////////////////////////////////////////////////////////////////////
            /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
            /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
            


            //////////////////Begin task 4,6  case 11000-11999 ///////////////////////////////////////////////////////////////////////////////////
            /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
            /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
            case 11000:
                sfc = 11010;
                break;
            
            case 11010:
                linefollow_msg.data = 1;    
                pub_linefollow_->publish(linefollow_msg);
                sfc = 11020;
                break;
            
            case 11020:
                if(status_linefollow == 20)
                { 
                    status_linefollow = 0;
                    sfc = 11030;
                }
                break;
            
            case 11030:
                linefollow_msg.data = 6;    
                pub_linefollow_->publish(linefollow_msg);
                sfc = 11040;
                break;
            
            case 11035:
                linefollow_msg.data = 1;
                pub_linefollow_->publish(linefollow_msg);
                sfc = 11040;
                break;
            
            case 11040: 
                  if(status_linefollow == 20)
                  {
                    status_linefollow = 0;
                    sfc = 11495;
                  }
                break;

            // Her kan nr 5 sættes ind hvis det er
            case 11495:
                linefollow_msg.data = 7;
                pub_linefollow_->publish(linefollow_msg);
                sfc = 11500;
                break;

            case 11500:
                //kør ned af bakken
                 linefollow_msg.data = 6;    
                pub_linefollow_->publish(linefollow_msg);
                sfc = 11505;
                break;
            case 11505:
                linefollow_msg.data = 2;
                pub_linefollow_->publish(linefollow_msg);
                sfc = 11999;
                break;
        
            case 11999:
                sfc = 12000;
                break;

            //////////////////END task 4,6 case 11000-11999 ///////////////////////////////////////////////////////////////////////////////////
            /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
            /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


            
            //////////////////Begin task 15 case 12000-12999 ///////////////////////////////////////////////////////////////////////////////////
            /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
            /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

            case 12000:
                sfc = 12100;

                break; 

            case 12100:
                    if(status_linefollow == 20)
                    {
                    status_linefollow = 0;
                    sfc = 11030;
                    }
                break;
            
            case 12110: // publish to robot arm to move its arm down to be able to touch buttom
                robot_msg.cmd = 6;
                robot_msg.pose = {0.29200,0.0,-0.12,0.0,1.45,0.0};
                pub_robot_->publish(robot_msg);
                m_lastTime1 = m_clock->now().seconds(); // start timer for timeout
                sfc = 12120;
                break;
            
            case 12120: // waitting for robot to go to pose
                if(robot_status == 1){ // check if robot is done
                    robot_status = 0;
                    robot_attempts = 0;
                    sfc = 12130;
                }
                m_lastTime2 = m_clock->now().seconds(); // get time now
                if((m_lastTime2-m_lastTime1) >15.0){ // if timeout 
                    RCLCPP_INFO(this->get_logger(), "timed out");
                    robot_attempts ++;
                    
                    sfc = 12110; // go back and resend the robot cmd
                    if(robot_attempts == 5){
                       
                       RCLCPP_INFO(this->get_logger(), "timed out, robot can't go to position");

                    }
                    
                }
                break;

            
            case 12130:
                linefollow_msg.data = 6;    
                pub_linefollow_->publish(linefollow_msg);
                // Maybe stop in code
                break;


            //////////////////END task 8 case 12000-12999 ///////////////////////////////////////////////////////////////////////////////////
            /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
            /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////




            /// maybe ///
            //////////////////Begin take ball from plato case 13000-13999 ///////////////////////////////////////////////////////////////////////////////////
            /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
            /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////




            //////////////////END take ball from plato case 13000-13999 ///////////////////////////////////////////////////////////////////////////////////
            /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
            /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////




            ////////////////// BEGIN Drive with navigation to task 4 case 14000-14999 ///////////////////////////////////////////////////////////
            /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
            /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

            case 14000:
                sfc = 14010;
                break;
            
            case 14010:
                pub_mobile_->publish(task_4_pose);

                sfc = 14020;
                
                break;
            
            case 14020:
                if(status_mobile == 1){
                    status_mobile = 0;
                    sfc = 14999;
                }

                break;
            
            case 14030:

                break;
            
            case 14040:

                break;
            
            case 14050:

                break;
            
            case 14060:

                break;
            
            case 14070:

                break;
            
            

            case 14999:
                sfc = 15000;
                break;

            //////////////////END Drive with navigation to task 4 case 14000-14999 //////////////////////////////////////////////////////////////
            /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
            /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////





            ////////////////// BEGIN  task 4 case 15000-15999 ///////////////////////////////////////////////////////////
            /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
            /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

            case 15000:
                sfc = 15010;
                break;
            
            case 15010:// start the line follower
                linefollow_msg.data = 1;
                pub_linefollow_->publish(linefollow_msg);
                sfc= 15020;
                break;
            
            case 15020: // waitting for robot to detect udflætningen tæt på task 5
                if (status_linefollow == 20){ //line following har fundet en sammenfletning eller skarpt sving og venter på en svar fra ros
                    status_linefollow = 0;
                    linefollow_msg.data = 0;
                    pub_linefollow_->publish(linefollow_msg);
                    sfc = 15030;
                }
                m_lastTime2 = m_clock->now().seconds();
                if ((m_lastTime2-m_lastTime1)> 90.0){
                    RCLCPP_INFO(this->get_logger(),"timed out");
                }
                break;
            
            case 15030: 
                sfc = 15999;
                break;
            
            case 15040:

                break;
            


            case 15999:
                sfc = 16000;
                break;
            ////////////////// END task 4 case 15000-15999 ///////////////////////////////////////////////////////////
            /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
            /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

            /// MAYBE 
            ////////////////// BEGIN  task 5 case 16000-16999 ///////////////////////////////////////////////////////////
            /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
            /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
            case 16000:
                sfc = 16999;
                break;
            
            case 16010:
                
                break;
            
            case 16020:

                break;

            case 16999:
                sfc = 17000;
                break;
            /// MAYBE
            ////////////////// END  task 5 case 16000-16999 ///////////////////////////////////////////////////////////
            /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
            /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


            ////////////////// BEGIN drive down ramp case 17000-17999 ///////////////////////////////////////////////////////////
            /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
            /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
            case 17000:
                sfc = 17010;
                break;
            
            case 17010: // send command to linefollower to begin driveing ige ud
                linefollow_msg.data = 1;
                pub_linefollow_->publish(linefollow_msg);
                sfc= 17020;
                break;
            
            case 17020: // waitting for the lige follower to detect the samflætningen before task 15
                if (status_linefollow == 17){ 
                    status_linefollow = 0;
                    linefollow_msg.data = 0;
                    pub_linefollow_->publish(linefollow_msg);
                    sfc = 17030;
                }
                m_lastTime2 = m_clock->now().seconds();
                if ((m_lastTime2-m_lastTime1)> 90.0){
                    RCLCPP_INFO(this->get_logger(),"timed out");
                }
                break;
            
            case 17030:
                sfc = 17999;
                break;

            case 17999:
                sfc = 18000;
                break;
            ////////////////// END drive down ramp case 17000-17999 ///////////////////////////////////////////////////////////
            /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
            /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

             ////////////////// BEGIN task 15 case 18000-18999 ///////////////////////////////////////////////////////////
            /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
            /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
            case 18000:
                sfc = 18010;
                break;

            case 18010: // publish to robot arm to move its arm down to be able to touch buttom
                robot_msg.cmd = 6;
                robot_msg.pose = {0.29200,0.0,-0.12,0.0,1.45,0.0};
                pub_robot_->publish(robot_msg);
                m_lastTime1 = m_clock->now().seconds(); // start timer for timeout
                sfc = 18020;
                break;
            
            case 18020: // waitting for robot to go to pose
                if(robot_status == 1){ // check if robot is done
                    robot_status = 0;
                    robot_attempts = 0;
                    sfc = 18020;
                }
                m_lastTime2 = m_clock->now().seconds(); // get time now
                if((m_lastTime2-m_lastTime1) >15.0){ // if timeout 
                    RCLCPP_INFO(this->get_logger(), "timed out");
                    robot_attempts ++;
                    
                    sfc = 18010; // go back and resend the robot cmd
                    if(robot_attempts == 5){
                       
                       RCLCPP_INFO(this->get_logger(), "timed out, robot can't go to position");

                    }
                    
                }
                break;
            
            case 18030: // send to begin the linefollower
                linefollow_msg.data = 1;
                pub_linefollow_->publish(linefollow_msg);
                sfc= 18040;
                break;

            case 18040: // waitting to line follower is done
                if (status_linefollow == 20){ //line following har fundet en sammenfletning eller skarpt sving og venter på en svar fra ros
                    status_linefollow = 0;
                    linefollow_msg.data = 0;
                    pub_linefollow_->publish(linefollow_msg);
                    sfc = 18050;
                }
                m_lastTime2 = m_clock->now().seconds();
                if ((m_lastTime2-m_lastTime1)> 90.0){
                    RCLCPP_INFO(this->get_logger(),"timed out");
                }
                break;
            
            case 18050:
                sfc = 18999;
                break;
            
            
            
            case 18999: 
                
                break;
            ////////////////// END task 15 case 18000-18999 ///////////////////////////////////////////////////////////
            /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
            /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

            default:
                break;
            }
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
        rclcpp::Publisher<std_msgs::msg::Int8>::SharedPtr pub_camera_ball_;
        int32_t status_aruco = 0;
        int32_t status_ball = 0;
        std_msgs::msg::Bool aruco_msg;
        std_msgs::msg::Int8 ball_msg;


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
        rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pub_mobile_relative_;
        rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr sub_mobile_;
        
        int32_t status_mobile = 0;
        double rot_r_lookup{}, rot_p_lookup{}, rot_y_lookup{};
        tf2::Quaternion quat_tf_lookup;
        tf2::Matrix3x3 matrix;

        /// the 3 packages trolly pose, make as vector
        geometry_msgs::msg::PoseStamped trolly_pose[3]; 
        /// the 3 house pose, make as vector
        geometry_msgs::msg::PoseStamped house_pose[3]; // red 0 // yelllow 1 // green 2



        // task 10 pose
        geometry_msgs::msg::PoseStamped task_10_pose;


        // task 4 pose
        geometry_msgs::msg::PoseStamped task_4_pose;


        // golfball Hole pose
        geometry_msgs::msg::PoseStamped golfball_sheach_pose;
        geometry_msgs::msg::PoseStamped golf_hole_pose;

        geometry_msgs::msg::PoseStamped golfball_release_pose;

        geometry_msgs::msg::PoseStamped speed;


        int32_t golfball_counter = 0;



        // line follow
        rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr sub_linefollow_;
        rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr pub_linefollow_;
        int32_t status_linefollow = 0;
        std_msgs::msg::Int32 linefollow_msg;


        // joystick
        // rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr sub_joy_;
        bool joy_status = true; 
        rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sub_joy_status_;


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