from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
def generate_launch_description():
    servo_init = LaunchConfiguration("angle_init",default="-90 -90 -90 -90 -90 -90")
    
    DeclareLaunchArgument("init_joint_state",default_value=servo_init)
    servo_com = Node(
        package="fs_servo",
        executable="servo_node"
        # parameters=servo_init
    )
    talker_node = Node(
        package="fs_servo",
        executable="joint_publisher"
    )
    return LaunchDescription([
        servo_com,
        talker_node]
    )