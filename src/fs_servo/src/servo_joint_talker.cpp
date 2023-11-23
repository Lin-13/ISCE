#include <rclcpp/rclcpp.hpp>
#include <string>
#include <chrono>
#include <math.h>
#include <unistd.h>
#include "servo_msgs/msg/joint_state.hpp"
#include "servo_msgs/msg/set_joint_angles.hpp"
#include "servo_msgs/srv/get_angles.hpp"
using namespace std::chrono_literals;
class TestPub : public rclcpp::Node{
public:
    TestPub(std::string topic_name):
        Node(topic_name){
            
            this->servo_angle_publisher_ = this->create_publisher<servo_msgs::msg::SetJointAngles>(
                "/fsservo/in/set_joint_angles",10);
            init_time_ = std::chrono::steady_clock::now();
            auto timer_set_cb = [this]() {
                RCLCPP_INFO(this->get_logger(),"Send Joint State to topic");
                this->servo_angle_publisher_->publish(gen_sin_wave());
                };
            timer_pub_cb_group = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
            timer_set_ = this->create_wall_timer(100ms,timer_set_cb,timer_pub_cb_group);
            client_cb_group = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
            this->get_angle_client_ = this->create_client<servo_msgs::srv::GetAngles>
                                            ("/fsservo/server/get_cur_angles",
                                            rmw_qos_profile_services_default,
                                            client_cb_group);
                                  
            timer_client_cb_group = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
            timer_get_ = this->create_wall_timer(1s,
                                            std::bind(&TestPub::timer_get_angles_cb,this),
                                            timer_client_cb_group);
        }
private:
    servo_msgs::msg::SetJointAngles gen_sin_wave(){
        auto time_now = std::chrono::steady_clock::now();
        std::chrono::steady_clock::duration t = time_now - init_time_;
        double t_sec = t.count()/1.0e9;
        servo_msgs::msg::SetJointAngles msg;
        double A = 90.0;
        msg.id = {0,1,2,3,4,5};
        msg.angles = std::vector<double>{
            A*sin(0.1 * t_sec),
            A*sin(0.2 * t_sec),
            A*sin(0.4 * t_sec),
            A*sin(0.4 * t_sec),
            A*sin(0.2 * t_sec),
            A*sin(0.1 * t_sec),
        };
        return msg;
    }
    void timer_get_angles_cb(){
        auto request = std::make_shared<servo_msgs::srv::GetAngles::Request>();
        request->id = std::vector<uint8_t> {0,1,2,3,4,5};
        auto result_future = this->get_angle_client_->async_send_request(request);
        RCLCPP_INFO(this->get_logger(),"Connecting server...");
        auto status = result_future.wait_for(500ms);
        if(status == std::future_status::ready){
            auto response  = result_future.get();
            RCLCPP_INFO(this->get_logger(),"Angle: %.2f\t%.2f\t%.2f\t%.2f\t%.2f\t%.2f",\
                                    response->angle[request->id[0]],\
                                    response->angle[request->id[1]],\
                                    response->angle[request->id[2]],\
                                    response->angle[request->id[3]],\
                                    response->angle[request->id[4]],\
                                    response->angle[request->id[5]]);
        }else if(status == std::future_status::deferred){
            RCLCPP_INFO(this->get_logger(),"Server Deferred.");
        }else if(status == std::future_status::timeout){
            RCLCPP_INFO(this->get_logger(),"Server Timeout.");
        }else{
            RCLCPP_INFO(this->get_logger(),"Undefined status:%d.", status);
        }
    }
    std::chrono::steady_clock::time_point init_time_;
    rclcpp::Publisher<servo_msgs::msg::SetJointAngles>::SharedPtr servo_angle_publisher_;
    rclcpp::Client<servo_msgs::srv::GetAngles>::SharedPtr get_angle_client_;
    rclcpp::TimerBase::SharedPtr timer_set_,timer_get_;
    rclcpp::CallbackGroup::SharedPtr timer_pub_cb_group,timer_client_cb_group,client_cb_group;
};
int main(int argc,char**argv){
    rclcpp::init(argc,argv);
    auto node = std::make_shared<TestPub>("joint_talker");
    rclcpp::executors::MultiThreadedExecutor executer;
    executer.add_node(node);
    executer.spin();
    rclcpp::shutdown();
}