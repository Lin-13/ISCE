#include <iostream>
#include <chrono>
#include <string>
#include <vector>
#include <map>
#include <unistd.h>
//Servo
//#include <CSerialPort/SerialPort.h>
#include <FashionStar/UServo/FashionStar_UartServo.h>
#include <FashionStar/UServo/FashionStar_UartServoProtocol.h>

//ROS
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include "servo_msgs/msg/joint_state.hpp"
#include "servo_msgs/msg/set_joint_angles.hpp"
#include "servo_msgs/srv/get_angles.hpp"

using namespace fsuservo;
using namespace std::chrono_literals;
using namespace std::placeholders;


class FSServo : public rclcpp::Node{
public:
    using servoIDType = uint8_t;
    using servoJointAngleType = double;
    using servoJointCurrentType = double;
    using servoIDArrayType = std::vector<servoIDType>;
    using servoJointAngleArrayType = std::vector<servoJointAngleType>;
    using servoJointCurrentArrayType = std::vector<servoJointCurrentType>;
    using servoTrajectoryPoint = std::vector<servoJointAngleArrayType>;
    FSServo(std::string servo_port,servoIDArrayType servo_id):
        servo_port_(servo_port),
        servo_id_(servo_id),
        proto_(servo_port,FSUS_DEFAULT_BAUDRATE),
        Node("fs_servo")
        {
            servo_init();
            //创建独立的互斥回调组
            cb_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
            server_cb_group = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
            joint_state_publisher_ = this->create_publisher<servo_msgs::msg::JointState>(
                                            "/fsservo/out/joint_state",
                                            10);
            rclcpp::SubscriptionOptions options;
            options.callback_group = cb_group_;
            joint_angles_set_listener_ = this->create_subscription<servo_msgs::msg::SetJointAngles>(
                                            "/fsservo/in/set_joint_angles",
                                            10,
                                            std::bind(&FSServo::set_joint_angles_topic_cb,this,_1),
                                            options);
            timer_ = this->create_wall_timer(50ms,std::bind(&FSServo::joint_timer_callback,this));
            get_cur_angles_server_ = this->create_service<servo_msgs::srv::GetAngles>(
                                            "/fsservo/server/get_cur_angles",
                                            std::bind(&FSServo::get_cur_angles_server_cb,this,_1,_2),
                                            rmw_qos_profile_services_default,
                                            server_cb_group);
            
    }
    void joint_timer_callback(){
        servo_msgs::msg::JointState joint_msg;
        
        std::vector<double> joint_state{0,0,0,0,0,0};
        joint_msg.id = servo_id_;
        joint_msg.angles = query_joint_angles(servo_id_);
        joint_msg.current = query_joint_current(servo_id_);
        RCLCPP_INFO(this->get_logger(),"Angle: %.2f\t%.2f\t%.2f\t%.2f\t%.2f\t%.2f",\
                                        joint_msg.angles[servo_id_[0]],\
                                        joint_msg.angles[servo_id_[1]],\
                                        joint_msg.angles[servo_id_[2]],\
                                        joint_msg.angles[servo_id_[3]],\
                                        joint_msg.angles[servo_id_[4]],\
                                        joint_msg.angles[servo_id_[5]]);
        this->joint_state_publisher_->publish(joint_msg);
    }
    void set_joint_angles_topic_cb(servo_msgs::msg::SetJointAngles::SharedPtr msg){
        // std::lock_guard<std::mutex> guard(serial_lock_);
        // for (auto& id : msg->id){
        //     this->servos_[id].setAngle(msg->angles[id]);
        // }
        set_joint_angles(msg->id,msg->angles);
    }
    void set_move_trajectory(const std::vector<double>& time_point,const servoTrajectoryPoint& trajectory){
        if(time_point.size() != trajectory.size()){
            RCLCPP_INFO(this->get_logger(),"Trajectory time point size not match!");
            return;
        }
        const static int delta_t_ms = 100;
        auto zero_time = std::chrono::steady_clock::now();
        double last_t = 0;
        std::vector<int> index;
        
        for(int index = 0; index < time_point.size(); index++){
            auto t = time_point[index];
            auto point = trajectory[index];
            auto current_joint = this->query_joint_angles(servo_id_);
            servoJointAngleArrayType angles_vel = {};
            for(int i =0; i < current_joint.size(); i++){
                angles_vel.push_back(point[i]-current_joint[i]/(t-last_t));//vel:deg/s
            }
            auto start = std::chrono::steady_clock::now();
            int cnt = 0;
            while(1){
                servoJointAngleArrayType angle_set = {};
                for(int i =0; i < point.size(); i++){
                    angle_set.push_back(cnt*delta_t_ms*angles_vel[i] + current_joint[i]);
                }
                this->set_joint_angles(angle_set);
                
                usleep(delta_t_ms*1000);
                cnt++;
                auto time_now = std::chrono::steady_clock::now();
                auto dur = time_now - zero_time;
                if(dur.count()/1.0e9 > t){//seconds
                    break;
                }
            }
        }
    }
    void get_cur_angles_server_cb(servo_msgs::srv::GetAngles::Request::SharedPtr request, 
                                servo_msgs::srv::GetAngles::Response::SharedPtr response){
        auto start = std::chrono::steady_clock::now();
        RCLCPP_INFO(this->get_logger(),"Servo Request:%d\t%d\t%d\t%d\t%d\t%d",
                                        request->id[0],
                                        request->id[1],
                                        request->id[2],
                                        request->id[3],
                                        request->id[4],
                                        request->id[5]);
        response->angle = query_joint_angles(request->id);
        auto end = std::chrono::steady_clock::now();
        auto duration = end - start;
        RCLCPP_INFO(this->get_logger(),"Servo Time Use:%.2f ms.",duration.count()/1.0e6);
        RCLCPP_INFO(this->get_logger(),"Response: %.2f\t%.2f\t%.2f\t%.2f\t%.2f\t%.2f",\
                                        response->angle[request->id[0]],\
                                        response->angle[request->id[1]],\
                                        response->angle[request->id[2]],\
                                        response->angle[request->id[3]],\
                                        response->angle[request->id[4]],\
                                        response->angle[request->id[5]]);
    }
    bool servo_init(){
        for(auto& id : servo_id_){
                servos_.insert(std::make_pair(id,new FSUS_Servo(id,&proto_)));
            }
            // bool success = servo_ping();
            //初始角度参数
            rcl_interfaces::msg::ParameterDescriptor descripter;
            descripter.name="init_joint_state";
            descripter.description = "Init Servo Joint Angles while setup.";
            servoJointAngleArrayType init_state {0,0,0,0,0,0};
            servoJointAngleArrayType state_ = this->declare_parameter("init_joint_state",
                                            init_state,
                                            descripter);
            set_joint_angles(state_);
            return 1;
    }

    void set_joint_angles(const servoIDArrayType& id,const servoJointAngleArrayType& angles){
        std::lock_guard<std::mutex> guard(serial_lock_);
        int size = id.size();
        for(int i = 0; i < size; i++){
            servos_[servo_id_[id[i]]]->setAngle(angles[i]);
            usleep(this->delay_us);
        }
    }
    //override
    void set_joint_angles(const servoJointAngleArrayType& angles){
        std::lock_guard<std::mutex> guard(serial_lock_);
        int cnt = 0,max_size = servo_id_.size();
        for(auto& angle : angles){
            if (cnt >= max_size){
                break;
            }
            servos_[servo_id_[cnt]]->setAngle(angle);
            usleep(this->delay_us);
            cnt++;
        }
    }
    servoJointAngleArrayType query_joint_angles(const servoIDArrayType& id){
        servoJointAngleArrayType joint_state;
        // std::lock_guard<std::mutex> guard(serial_lock_);
        for(auto& i:id){
            joint_state.push_back(servos_[i]->queryAngle());
        }
        return joint_state;
    }
    servoJointAngleArrayType query_joint_current(servoIDArrayType& id){
        // std::lock_guard<std::mutex> guard(serial_lock_);
        servoJointAngleArrayType joint_state;
        for(auto& i:id){
            joint_state.push_back(servos_[i]->queryCurrent());
        }
        return joint_state;
    }
    void update_joint_state(void){
        // std::lock_guard<std::mutex> guard(serial_lock_);
        for(auto& [id,servo] : servos_){
            joint_state_angles_[id] = servo->queryAngle();
            joint_state_currents_[id] = servo->queryCurrent();
        }
    }
    bool servo_ping(){
        bool success = true;
        for(auto& [id,servo] : servos_){
            if (! servo->ping()){
                success = false;
            }
        }
        return success;
    }
private:

    std::string servo_port_;
    servoIDArrayType servo_id_;
    FSUS_Protocol proto_;
    std::map<servoIDType,FSUS_Servo*> servos_;
    std::map<servoIDType,servoJointAngleType> joint_state_angles_;
    std::map<servoIDType,servoJointCurrentType> joint_state_currents_;
    std::mutex serial_lock_;
    int delay_us = 2e4; //传送指令时的间隔

    rclcpp::CallbackGroup::SharedPtr cb_group_,server_cb_group;
    rclcpp::Publisher<servo_msgs::msg::JointState>::SharedPtr joint_state_publisher_;
    rclcpp::Subscription<servo_msgs::msg::SetJointAngles>::SharedPtr joint_angles_set_listener_;
    rclcpp::Service<servo_msgs::srv::GetAngles>::SharedPtr get_cur_angles_server_;
    rclcpp::TimerBase::SharedPtr timer_;
};
int main(int argc,char** argv){
    rclcpp::init(argc,argv);

    std::string servo_port = "/dev/ttyUSB0";
    FSServo::servoIDArrayType servo_id = {0,1,2,3,4,5};
    FSServo::SharedPtr fs_servo_node = std::make_shared<FSServo>(servo_port,servo_id);
    rclcpp::executors::SingleThreadedExecutor executer;
    executer.add_node(fs_servo_node);
    executer.spin();
    rclcpp::shutdown();
    return 0;
}