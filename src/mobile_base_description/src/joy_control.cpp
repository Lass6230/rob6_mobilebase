#include <sensor_msgs/msg/joy.hpp>
#include <std_msgs/msg/bool.hpp>
#include "rclcpp/rclcpp.hpp"


























using std::placeholders::_1;


class JoyListner : public rclcpp::Node
{
    public:
        JoyListner() : Node("Joy_listner")
        {
            sub_joy_ = this->create_subscription<sensor_msgs::msg::Joy>("/joy",10,std::bind(&JoyListner::subJoyStatus, this, _1));
            pub_joy_status_ = this->create_publisher<std_msgs::msg::Bool>("/joy_status",10);
        }
    
    private:
        void subJoyStatus(const sensor_msgs::msg::Joy & msg){
            if(msg.buttons[0]){
                if(on == false){
                    status_msg.data = true;
                    on = true;
                    off = false;
                    pub_joy_status_->publish(status_msg);
                }
            }
            if(msg.buttons[1]){
                if(off == false){
                    status_msg.data = false;
                    on = false;
                    off = true;
                    pub_joy_status_->publish(status_msg);
                }
            }
        }
        // joystick
        rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr sub_joy_;
        rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr pub_joy_status_;
        std_msgs::msg::Bool status_msg;
        int32_t joy_status = 1;
        bool on = false;
        bool off = true;

};





int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::executors::MultiThreadedExecutor executor;
  auto joy_listner_program = std::make_shared<JoyListner>();
  executor.add_node(joy_listner_program);
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "JoyListner Ready.");
  executor.spin();
  
  
  rclcpp::shutdown();
}