from time import sleep
import time
import numpy as np
import os
import roboticstoolbox as rtb
from roboticstoolbox import ERobot, ctraj
import matplotlib.pyplot as plt
from scipy.stats import gaussian_kde
from typing import Union
from launch_ros.substitutions import FindPackageShare

from .utils import *
from .hyper_robot import HyperRobot
class Voxel():
    #x,y,z 为np.linspace,必须单调递增
    #根据3维点云构建voxel，保存该体素内的点数量以及一维直（平均）
    def __init__(self,x:np.ndarray,y:np.ndarray,z:np.ndarray,pt:np.ndarray):
        assert 3 <= pt.shape[1] <= 4 and len(pt.shape) == 2
        self.x = x
        self.y = y
        self.z = z
        self.pt = pt
        self.store_value = (pt.shape[1] == 4)
        self.voxels = np.zeros((len(self.x),len(self.y),len(self.z)))
        self.values = np.zeros((len(self.x),len(self.y),len(self.z)))#avg value of pt[:3] in voxels
        self.update(self.pt)
    def update(self,pt:np.ndarray):
        assert 3 <= pt.shape[1] <= 4 and len(pt.shape) == 2
        x_num,y_num,z_num = len(self.x),len(self.y),len(self.z)
        x_start,y_start,z_start = self.x[0],self.y[0],self.z[0]
        x_end,y_end,z_end = self.x[-1],self.y[-1],self.z[-1]
        # x,y,z = np.indices((x_num,y_num,z_num))
        voxels = np.zeros((x_num,y_num,z_num))
        self.store_value = (pt.shape[1] == 4)
        for pk in self.pt:
            x_k = int((pk[0] - x_start) * x_num / (x_end - x_start))
            y_k = int((pk[1] - y_start) * y_num / (y_end - y_start))
            z_k = int((pk[2] - z_start) * z_num / (z_end - z_start))
            if x_k > 0 and x_k < x_num and y_k > 0 and y_k < y_num and z_k >0 and z_k < z_num: 
                self.voxels[x_k,y_k,z_k] += 1
                if self.store_value:
                    self.values[x_k,y_k,z_k] = self.values[x_k,y_k,z_k] * (self.voxels[x_k,y_k,z_k] - 1)/ self.voxels[x_k,y_k,z_k] + pk[3] / self.voxels[x_k,y_k,z_k]
    def index(self,point: Union[list, tuple, np.ndarray],normalize = False):
        num = 0
        index = []
        x_k = int((point[0] - self.x[0]) * len(self.x) / (self.x[-1] - self.x[0]))
        y_k = int((point[1] - self.y[0]) * len(self.y) / (self.y[-1] - self.y[0]))
        z_k = int((point[2] - self.z[0]) * len(self.z) / (self.z[-1] - self.z[0]))
        if 0<=x_k<len(self.x)  and 0<=y_k<len(self.y) and 0<=z_k<len(self.z): 
            num = self.voxels[x_k,y_k,z_k]
            index = [x_k,y_k,z_k]
        if normalize:
            num = num / len(self.pt)
        return index,num,self.values[x_k,y_k,z_k]
    def density(self):
        kde_x = gaussian_kde(self.pt[:,0])
        kde_y = gaussian_kde(self.pt[:,1])
        kde_z = gaussian_kde(self.pt[:,2])
        density_x = kde_x(self.x).reshape(-1,1)
        density_y = kde_y(self.y).reshape(-1,1)
        density_z = kde_z(self.z).reshape(-1,1)
        return np.hstack([density_x,density_y,density_z])
    def plot(self):
        print(f"Voxel:{self.voxels.shape}")
        fig = plt.figure("Voxel",figsize=(16,12))
        ax = fig.add_subplot(111, projection='3d')
        ax.voxels(self.voxels)
        # 设置坐标轴标签
        ax.set_xlabel('X')
        ax.set_ylabel('Y')
        ax.set_zlabel('Z')

        # 显示图形
        plt.show()
class RobotSpace():
    def __init__(self,pkg_name:str="hyper_robot3",num_voxel=20,sample = 10000):
        self.robot = HyperRobot(pkg_name)
        # self.hyper_ets = self.hyper.ets()
        self.sample = sample
        self.q,self.pt = self.monte_karlo(sample)
        self.pt = np.hstack([self.pt,self.evalute_q()])
        self.num_voxel = num_voxel
        self.voxels = Voxel(np.linspace(-0.5,0.5,self.num_voxel),
                            np.linspace(-0.5,0.5,self.num_voxel),
                            np.linspace(-1.0,0.0,self.num_voxel),
                            self.pt)
    def monte_karlo(self,N=10000):
        q_random = self.robot.ets.random_q(N)
        p = self.robot.ets.fkine(q_random)
        self.pt = p.t
        return q_random,p.t
    def evalute_q(self,max = 100):
        tr_j = []
        yo = []
        for qk in self.q:
            jacob_k = self.robot.ets.jacob0_analytical(qk)
            y = np.sqrt(np.linalg.det(jacob_k@(jacob_k.T)))
            yo.append(y if y<max else max)
            # tr_j.append(jacob_k)
        yo = np.array(yo).reshape((-1,1))
        return yo
# test()
def main():
    hyper =RobotSpace(num_voxel=20,sample=100000)
    # hyper.voxels.plot()
    plt.figure("Jacob evalute",figsize=(32,32))
    plt.subplot(3,1,1)
    plt.plot(hyper.voxels.x,np.sum(hyper.voxels.values,(1,2)))
    plt.xlabel("X")
    plt.subplot(3,1,2)
    plt.plot(hyper.voxels.y,np.sum(hyper.voxels.values,(0,2)))
    plt.xlabel("Y")
    
    plt.subplot(3,1,3)
    plt.plot(hyper.voxels.z,np.sum(hyper.voxels.values,(0,1)))
    plt.xlabel("Z")
    plt.show()
    hyper.voxels.plot()
if __name__ == "__main__":
    main()