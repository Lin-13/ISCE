import rclpy
from rclpy.node import Node
from servo_msgs.msg import SetJointAngles
from sensor_msgs.msg import JointState

import numpy as np
import os
import roboticstoolbox as rtb
from roboticstoolbox import ERobot, ctraj
import matplotlib.pyplot as plt
from spatialmath import SE3
from roboticstoolbox.backends.swift import Swift
from roboticstoolbox.tools.trajectory import jtraj,quintic,mstraj

from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution

from .hyper_robot import HyperRobot
class ControlJointPub(Node):
    def __init__(self,time_period = 0.1):
        super().__init__("joint_pub") # type: ignore
        self.time_period = time_period
        self.joint_publisher = self.create_publisher(SetJointAngles,
                                                     "/fsservo/in/set_joint_angles",
                                                     10)

        self.timer = self.create_timer(time_period,self.timer_callback)
    def timer_callback(self):
        msg = SetJointAngles()
        msg.id = [0,1,2,3,4,5]
        msg.angles = [0]*6
        self.joint_publisher.publish(msg)
        self.get_logger().info(f"Send angle:{msg.angles}")
class VirtualControlJointPub(Node):
    def __init__(self,time_period = 0.5):
        super().__init__("joint_pub") # type: ignore
        self.index = 0
        self.robot = HyperRobot()
        self.traj = self.robot.generate_traj()
        self.time_period = time_period
        self.joint_publisher = self.create_publisher(JointState,
                                                     "/joint_states",
                                                     10)

        self.timer = self.create_timer(time_period,self.timer_callback)
    
    def timer_callback(self):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        # msg.name = ["Joint0","Joint1","Joint2","Joint3","Joint4","Joint5",
        #             "JointPen","JointPenRoll","JointPenPitch"]
        msg.name = self.robot.joint_name
        if self.index >= len(self.traj):
            if self.index == len(self.traj):
                self.index+=1
                print("Done")
            return
        msg.position = self.traj[self.index,:].tolist()
        self.index+=1
        self.joint_publisher.publish(msg)
        # self.get_logger().info(f"Send angle:{msg.position}")
def main(args=None):
    rclpy.init(args=args)
    pub = VirtualControlJointPub(0.05)
    rclpy.spin(pub)
    rclpy.shutdown()
    