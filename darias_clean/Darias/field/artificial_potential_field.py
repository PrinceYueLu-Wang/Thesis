
import math
import random
from matplotlib import pyplot as plt
from matplotlib.patches import Circle
import time
import numpy as np


class Vector3d():
    """
    3维向量, 支持加减, 支持常量乘法(右乘)
    """

    def __init__(self, x,y,z):
        self.deltaX = x
        self.deltaY = y
        self.deltaZ = z

        self.Vector3dUpdate()

    def Vector3dUpdate(self):

        self.vector_np= np.array([self.deltaX,
                                  self.deltaY,
                                  self.deltaZ])

        self.norm=np.linalg.norm(self.vector_np)

        if self.norm == 0 :
            self.unitvector_np=np.array([0.,0.,0.])
        else:
            self.unitvector_np = self.vector_np/self.norm

        self.unitvetor_x=self.unitvector_np[0]
        self.unitvetor_y=self.unitvector_np[1]
        self.unitvetor_z=self.unitvector_np[2] 

    def __add__(self, other):
        """
        + 重载
        :param other:
        :return:
        """
        vec = Vector3d(self.deltaX, self.deltaY,self.deltaZ)
        vec.deltaX += other.deltaX
        vec.deltaY += other.deltaY
        vec.deltaZ += other.deltaZ
        vec.Vector3dUpdate()
        return vec

    def __sub__(self, other):
        vec = Vector3d(self.deltaX, self.deltaY,self.deltaZ)
        vec.deltaX -= other.deltaX
        vec.deltaY -= other.deltaY
        vec.deltaZ -= other.deltaZ
        vec.Vector3dUpdate()
        return vec

    def __mul__(self, other):
        vec = Vector3d(self.deltaX, self.deltaY,self.deltaZ)
        vec.deltaX *= other
        vec.deltaY *= other
        vec.deltaZ *= other
        vec.Vector3dUpdate()
        return vec

    def __truediv__(self, other):
        return self.__mul__(1.0 / other)

    def __repr__(self):

        tmp = ''' Vector: \n deltaX:{} \n deltaY:{} \n deltaZ:{} \n norm : {} '''.format(self.deltaX, self.deltaY, self.deltaZ, self.norm)

        return tmp

class APF():
    """
    人工势场寻路
    """

    def __init__(self, start: (), goal: (), obstacles: [], k_att: float, k_rep: float, rr: float,
                 step_size: float, max_iters: int, goal_threshold: float, is_plot=False):
        """
        :param start: 起点
        :param goal: 终点
        :param obstacles: 障碍物列表，每个元素为Vector3d对象
        :param k_att: 引力系数
        :param k_rep: 斥力系数
        :param rr: 斥力作用范围
        :param step_size: 步长
        :param max_iters: 最大迭代次数
        :param goal_threshold: 离目标点小于此值即认为到达目标点
        :param is_plot: 是否绘图
        """

        self.start = Vector3d(start[0],start[1],start[2])
        self.current_pos = Vector3d(start[0], start[1],start[2])
        self.goal = Vector3d(goal[0],goal[1],goal[2])

        self.obstacles = [Vector3d(OB[0], OB[1],OB[2]) for OB in obstacles]

        self.k_att = k_att
        self.k_rep = k_rep
        self.rr = rr

        self.step_size = step_size
        self.max_iters = max_iters
        
        self.iters = 0

        self.goal_threashold = goal_threshold

        self.path = list()

        self.is_path_plan_success = False

        self.is_plot = is_plot

        self.delta_t = 0.01

    def attractive(self):
        """
        引力计算
        :return: 引力
        """
        att = (self.goal - self.current_pos) * self.k_att  # 方向由机器人指向目标点
        return att

    def repulsion(self):

        """
        斥力计算
        :return: 斥力大小
        """

        rep = Vector3d(0,0,0)  # 所有障碍物总斥力

        for obstacle in self.obstacles:
            # obstacle = Vector3d(0, 0)
            t_vec = self.current_pos - obstacle

            if (t_vec.norm > self.rr):  # 超出障碍物斥力影响范围              
                pass
            else:
                rep += Vector3d(t_vec.unitvetor_x, t_vec.unitvetor_y,t_vec.unitvetor_z) * self.k_rep * (
                        1.0 / t_vec.norm - 1.0 / self.rr) / (t_vec.norm ** 2)  # 方向由障碍物指向机器人
        return rep

    def path_plan(self):
        """
        path plan
        :return:
        """
        while (self.iters < self.max_iters and (self.current_pos - self.goal).norm > self.goal_threashold):
            # 循环未超过限制 且 还未到终点
            #             
            f_vec = self.attractive() + self.repulsion()

            self.current_pos += Vector3d(f_vec.unitvetor_x, f_vec.unitvetor_y,f_vec.unitvetor_z) * self.step_size
            self.iters += 1
            self.path.append([self.current_pos.deltaX, self.current_pos.deltaY,self.current_pos.deltaZ])

        if (self.current_pos - self.goal).norm <= self.goal_threashold:

            self.is_path_plan_success = True


if __name__ == '__main__':
    # 相关参数设置
    k_att, k_rep = 1.0, 100.0
    rr = 4
    step_size, max_iters, goal_threashold = .2, 500, .2  # 步长0.5寻路1000次用时4.37s, 步长0.1寻路1000次用时21s
    step_size_ = 2

    # 设置、绘制起点终点
    start, goal = (0,0,0), (15,15,15)
    is_plot = False

    # 障碍物设置及绘制

    obs = [[1, 4,1], [2, 4,2], [3,3,3], [6, 1,6], [6, 7,6], [10, 6,8], [11, 12,11], [14, 14,14]]
    print('obstacles: {0}'.format(obs))
    for i in range(0):
        obs.append([random.uniform(2, goal[1] - 1), random.uniform(2, goal[1] - 1)])
    # path plan

    if is_plot:
        apf = APF(start, goal, obs, k_att, k_rep, rr, step_size, max_iters, goal_threashold, is_plot)
    else:
        apf = APF(start, goal, obs, k_att, k_rep, rr, step_size, max_iters, goal_threashold, is_plot)
    apf.path_plan()

    if apf.is_path_plan_success:
        path = apf.path
        path_ = []
        i = int(step_size_ / step_size)
        while (i < len(path)):
            path_.append(path[i])
            i += int(step_size_ / step_size)

        if path_[-1] != path[-1]:  # 添加最后一个点
            path_.append(path[-1])

        print('planed path points:{}'.format(path_))
        print('path plan success')

    else:
        print('path plan failed')

    for item in apf.path:
        print("{}  {}  {}  ".format(item[0],item[1],item[2]))

    fig=plt.figure(1)
    ax = plt.axes(projection='3d')
    ax.set_xlabel('x')
    ax.set_ylabel('y')
    ax.set_zlabel('z')

    path_x=[item[0] for item in apf.path]
    path_y=[item[1] for item in apf.path]
    path_z=[item[2] for item in apf.path]

    ax.scatter3D(
            path_x[::1]    ,
            path_y[::1]    , 
            path_z[::1]    , 
            # c=self.time[::10]  , 
            # cmap='Reds'
            )
    for item in obs:
        
        u, v = np.mgrid[0:2*np.pi:20j, 0:np.pi:10j]
        radius=1.5
        x = np.cos(u)*np.sin(v) * radius + item[0]
        y = np.sin(u)*np.sin(v) * radius + item[1]
        z = np.cos(v) * radius + item[2] 
        ax.plot_wireframe(x, y, z, color="r")



    plt.show()

