#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <std_msgs/msg/string.hpp>
#include <string>
using namespace std::chrono_literals;
int main(int argc,char**argv){
    rclcpp::init(argc,argv);
    auto node = std::make_shared<rclcpp::Node>("test");
    auto publisher = node->create_publisher<std_msgs::msg::String>("test",10);
    auto param = node->declare_parameter<std::string>("cnt","1");
    auto timer = node->create_wall_timer(1s,[&publisher,&node](void){
        auto param = node->get_parameter("cnt");
        param.get_value<std::string>();
        std_msgs::msg::String str;
        str.data = "Hello";
        publisher->publish(str);
        RCLCPP_INFO(node->get_logger(),str.data+param.get_value<std::string>());
        return;
    });
    
    rclcpp::spin(node);
    rclcpp::shutdown();
}