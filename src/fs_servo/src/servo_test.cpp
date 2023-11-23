//
// Created by lwx on 23-10-22.
//
#include <iostream>
#include <unistd.h>
#include <string>
#include <map>
#include <unistd.h>
//Servo
//#include <CSerialPort/SerialPort.h>
#include <FashionStar/UServo/FashionStar_UartServo.h>
#include <FashionStar/UServo/FashionStar_UartServoProtocol.h>
//Ros
#include <rclcpp/rclcpp.hpp>
//
#include "servo_msgs/msg/joint_state.hpp"
#include "servo_msgs/msg/set_joint_angles.hpp"
#include "servo_msgs/srv/get_angles.hpp"
using namespace fsuservo;

class TestTask{
public:
	/*
	 *  proto:舵机协议对象
	 */
	TestTask(std::string servo_port,std::vector<int> servo_id):
			servo_port_(servo_port),
			servo_id_(servo_id),
			proto_(servo_port,FSUS_DEFAULT_BAUDRATE)
	{
		for(auto& id : servo_id_){
			servos_.insert(std::make_pair(id,FSUS_Servo(id,&proto_)));
		}
	}
	void testPing(){
		for(auto& [id,servo] : servos_){
			bool is_online = servo.ping();
			if(is_online){
				std::cout << "Servo id:" << id << " Online" << std::endl;
			}else{
				std::cout << "Servo id:" << id << " Not Online" << std::endl;
			}
		}
	}
	void testQueryAngles(){
		for(auto& [id,servo]:servos_){
			std::cout << "Servo id:" << id << "\tAngle:" << servo.queryAngle() << std::endl;
		}
	}
	void testSetAnglesZero(){
		for(auto& [id,servo]:servos_){
			servo.setAngle(0.0);
			usleep(2000);
			std::cout << "Servo id:" << id << "\tSet Angle:" << servo.queryAngle() << std::endl;
		}
	}
	void testSetAngles(){
		for(auto& [id,servo]:servos_){
			servo.setAngle(90.0);
			usleep(2000);
			std::cout << "Servo id:" << id << "\tSet Angle:" << servo.queryAngle() << std::endl;
		}
	}
	/*
	 * \brief : not work
	 */
	void testAnglesMturn(){
		for(auto& [id,servo]:servos_){
			servo.setRawAngleMTurn(90.0+360);
			std::cout << "Servo id:" << id << "\tSet Angle:" << servo.queryAngle() << std::endl;
		}
	}
	void testWeel(){
		for(auto& [id,servo]:servos_){
//			servo.wheelRunNCircle(true,3);
//			servo.setDamping();
			servo.wheelStop();
			servo.wheelRunNTime(true,5000);
			sleep(5);

			std::cout << "Servo id:" << id << "\tLatest Angle:" << servo.queryAngle() << std::endl;
		}
	}
	void testQueryCurrent(){
		for(auto& [id,servo]:servos_){
			std::cout << "Servo id:" << id << "\tCurrent:" << servo.queryCurrent() << " A." << std::endl;
		}
	}
private:
	std::string servo_port_;
	std::vector<int> servo_id_;
	FSUS_Protocol proto_;
	std::map<int,FSUS_Servo> servos_;
};


int main(int argc,char**argv){
	std::string servo_port = "/dev/ttyUSB0";
	std::vector<int> servo_id = {0,1,2,3,4,5};
	TestTask task(servo_port,servo_id);
	task.testPing();
	task.testQueryAngles();
	task.testSetAngles();
	sleep(5);
	task.testSetAnglesZero();
//	task.testWeel();
	task.testQueryAngles();
	task.testQueryCurrent();
}