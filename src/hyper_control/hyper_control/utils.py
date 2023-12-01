import numpy as np
from matplotlib import pyplot as plt
from scipy.stats import gaussian_kde
import time
import os
import roboticstoolbox as rtb
from roboticstoolbox import ERobot
from launch_ros.substitutions import FindPackageShare
# KDE方法估算概率密度函数,xyz分别估算
def plot_kde(q:np.ndarray,label:list = ["x","y","z"]):
    assert len(q.shape) ==2
    N,n = q.shape

    # x = np.linspace(-0.5,0.5,100)
    # y = np.linspace(-0.5,0.5,100)
    # z = np.linspace(-1.0,0.0,100)
    # axes = [x,y,z]
    fig = plt.figure(figsize=(16,12))
    M = 100 #
    res = np.zeros((M,n*2))
    header = []
    for s in label:
        header.append(s)
        header.append(s+"_density")
    header = ",".join(header)
    for i in range(n):
        ax = plt.subplot(n,1,i+1)
        kde = gaussian_kde(q[:,i])
        x = np.linspace(q[:,i].min(),q[:,i].max(),M)
        density = kde(x)
        res[:,i*2] = x
        res[:,i*2+1] = density
        ax.plot(x, density)
        plt.xlabel(label[i])
        plt.ylabel('Density')
    plt.show()
    print(res.shape)
    np.savetxt("./data/destiny.txt",res,fmt='%.2e',header=header)
    print("Save!")
    return res
# q hist
#param:q:[N,n] ndarray
def plot_hist(q:np.ndarray,title = str()):
    assert len(q.shape) ==2
    N,n = q.shape
    fig = plt.figure(figsize=(16,12))
    plt.title(title)
    for i in range(n):
        ax = plt.subplot(n,1,i+1)
        ax.hist(q[:,i],bins=50)
        
#pt:[N,3] array
def plot_scatter(p_t):
    fig = plt.figure(figsize=(16,12))
    ax = fig.add_subplot(111, projection='3d')
    ax.scatter(p_t[:,0], p_t[:,1], p_t[:,2])
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')
    plt.show()
#pt:[N,3] array
def plot_voxel(p_t):
    x_num,y_num,z_num = 20,20,20
    x_start,y_start,z_start = -0.5,-0.5,-1
    x_end,y_end,z_end = 0.5,0.5,0
    # x,y,z = np.indices((x_num,y_num,z_num))
    voxels = np.zeros((x_num,y_num,z_num))
    for pk in p_t:
        x_k = int((pk[0] - x_start) * x_num / (x_end - x_start))
        y_k = int((pk[1] - y_start) * y_num / (y_end - y_start))
        z_k = int((pk[2] - z_start) * z_num / (z_end - z_start))
        if x_k > 0 and x_k < x_num and y_k > 0 and y_k < y_num and z_k >0 and z_k < z_num: 
            voxels[x_k,y_k,z_k] = 1
    fig = plt.figure(figsize=(16,12))
    ax = fig.add_subplot(111, projection='3d')
    ax.voxels(voxels)

    # 设置坐标轴标签
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')

    # 显示图形
    plt.show()
    
def test():
    pkg_name = "hyper_robot3"
    pkg = FindPackageShare(pkg_name).find(pkg_name)
    model_path = os.path.join(pkg,"urdf", pkg_name + ".urdf")
    print(f"Get model from:{model_path}")
    hyper = ERobot.URDF(model_path)
    hyper_ets = hyper.ets()
    print("Rand Sample now.")
    print(f"Robot q limit:\n{hyper.qlim}")

    N = 10000
    start = time.time()
    q_random = hyper_ets.random_q(N)
    end = time.time()
    print(f"Generate {N} sample soints,use {end - start:.2f} seconds.")
    plot_hist(q_random,"Joint Angles Samples")

    print("Generating pose.")
    start = time.time()
    p00 = hyper_ets.fkine(q_random)
    p_t = p00.t
    end = time.time()
    print(f"Calculate pose from {N} sampled points,use {end - start:.2f} seconds.")
    plot_hist(p_t,"End Effector Pos")
    plot_kde(p_t)
    plot_voxel(p_t)