import rclpy
from rclpy.node import Node
from std_msgs.msg import String

def main(args = None):
    rclpy.init(args=args)
    node = Node("testpy")
    node.create_subscription(String,"test",
                             lambda str: node.get_logger().info("listener:"+str),
                             10)
    rclpy.spin(node)
    rclpy.shutdown()
    