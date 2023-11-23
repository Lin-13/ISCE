cmake_minimum_required(VERSION 3.10)
project(fs_servo)
set(CMAKE_CXX_STANDARD 17)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
#msgs
# list(APPEND CMAKE_PREFIX_PATH "${CMAKE_PROGRAM_PATH}/../../install") 
find_package(servo_msgs REQUIRED)

# rosidl_generate_interfaces(
#         ${PROJECT_NAME}
#         "msg/Joint.msg"
#         "srv/GetAngle.srv"
#         DEPENDENCIES
#         geometry_msgs
# )
# ament_export_dependencies(rosidl_default_runtime)
add_executable(servo_test src/servo_test.cpp)
add_executable(servo_node src/servo_publisher.cpp)
add_executable(joint_publisher src/servo_joint_talker.cpp)
#使本包的代码可以使用msg等
# rosidl_target_interfaces(servo_test ${PROJECT_NAME} "rosidl_typesupport_cpp")

target_link_libraries(servo_test  fsuartservo cserialport)
target_link_libraries(servo_node fsuartservo cserialport)
#ROS
ament_target_dependencies(servo_test rclcpp std_msgs servo_msgs)
ament_target_dependencies(servo_node rclcpp std_msgs servo_msgs)
ament_target_dependencies(joint_publisher rclcpp std_msgs servo_msgs)
# install(
#         TARGETS
#         servo_test
#         DESTINATION
#         ${CMAKE_HOME_DIRECTORY}/../../bin
# )
install(
        TARGETS
        servo_test
        servo_node
        joint_publisher
        DESTINATION
        lib/${PROJECT_NAME}
)
install(
        DIRECTORY
        launch
        DESTINATION
        share/${PROJECT_NAME}
)
ament_package()