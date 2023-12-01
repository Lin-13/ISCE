from unittest import loader
import numpy as np
import os
from time import sleep
import roboticstoolbox as rtb
from roboticstoolbox import ERobot, ctraj
import matplotlib.pyplot as plt
from spatialmath import SE3
from roboticstoolbox.backends.swift import Swift
from roboticstoolbox.tools.trajectory import jtraj,quintic,mstraj
import yaml
from .utils import plot_scatter
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution
class HyperRobot():
    def __init__(self,pkg_name:str="hyper_robot4"):
        self.pkg_name = pkg_name
        pkg = FindPackageShare(self.pkg_name).find(self.pkg_name)
        model_path = os.path.join(pkg,"urdf", self.pkg_name + ".urdf")
        self.hyper = ERobot.URDF(model_path)
        self.ets = self.hyper.ets()
        config_path = os.path.join(pkg,"config","joint_names_"+pkg_name+".yaml")
        file = open(config_path)
        config = yaml.safe_load(file)
        self.joint_name = config['controller_joint_names']
        print(f"Joint:{self.joint_name}")
    def generate_traj(self):
        rot = SE3()
        # rot = rot.RPY(-np.pi/2,0,-np.pi/2) #hyper_robot3
        p0 = SE3( 0.1,-0.1,-0.35)* rot
        p1 = SE3( 0.1, 0.0,-0.35)* rot
        p2 = SE3( 0.1, 0.1,-0.35)* rot
        p3 = SE3( 0.0, 0.1,-0.35)* rot
        p4 = SE3(-0.0, 0.1,-0.35)* rot
        p5 = SE3(-0.0, 0.0,-0.35)* rot
        # hold = SE3(-0.5, 0.0,-0.45)* rot
        p6 = SE3(-0.0,-0.1,-0.35)* rot
        p7 = SE3(-0.0,-0.1,-0.35)* rot
        p = SE3([p0,p1,p2,p3,p4,p5,p6,p7,p0])
        q = self.ets.ikine_LM(p)
        tr = mstraj(q.q,0.05,0.5,tsegment=[1,]*8)
        plt.figure("Traj",figsize=(32,32))
        plt.plot(tr.q)
        plt.figure("Location",figsize=(32,32))
        plt.show()
        p = self.ets.fkine(q.q)
        plot_scatter(p.t)
        trq = tr.q
        return trq
    def simulate(self):
        traj = self.generate_traj()
        dt = 0.05
        env = Swift()
        env.launch(realtime=False)
        env.add(self.hyper)
        for qk in traj:
            self.hyper.q = qk
            env.step(dt)
            sleep(dt)
        env.hold()
def main():
    robot = HyperRobot("hyper_robot4")
    robot.simulate()