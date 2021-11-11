import math
import random
from matplotlib import pyplot as plt
from matplotlib.patches import Circle
import time
import numpy as np
import quaternion

from Darias.controller import ControllSpeed_eef

class APF():

    def __init__(self):

        self.ParameterConfig()

        self.FieldConfig()

    def ParameterConfig(self):

        ##===========================#
        self.joint_num=7
        ##===========================#

        self.k_att = 1.0
        self.k_rep = 0.05

        self.step_size = 1

        self.activate_threshold = 0.05

        self.goal_threashold = 3*1e-3

        ##===========================#

        self.obstacle_num=0

    def FieldConfig(self):

        ## Obstacle group config 
        ## group of bubbles from octomap also with radius

        x=1.0+ np.arange(-5,6)*0.1
        z=1.42+ np.arange(-5,6)*0.1
        y=0.54

        sphere_wall=np.array(np.meshgrid(x,y,z)).T.reshape(-1,3)

        ## shape (x,3) x refers to num of bubbles

        self.obs_spherecenter = sphere_wall

        self.obstacle_num= sphere_wall.shape[0]

        ## here ball with radius 0.025m

        self.obstacle_radius = np.ones((self.obstacle_num,)) * 0.025
        
    
    def UpdateJointState(self,q_xyz,q_jInv):

        self.joints_xyz=q_xyz

        self.joints_jacobInv=q_jInv
        
        self.dist_matrix=self.DistanceCalculate(q_xyz,self.obs_spherecenter)


    def UpdateMapInfo(self,octree):
        pass

    def UpdateAttForce(self,att_force):
        self.att_force = att_force

    # def UpdateRepForce(self):

        # #=================#
        # #find 10 closest point to 7 joints
        # #if input less than 10 , then input size
        
        # rank = min(self.obstacle_num,10)

        # #每行前10小

        # selector_index = np.argpartition(self.dist_matrix,
        #                            kth=rank,
        #                            axis=1)[:,:rank]

        
        # selector=[[] for x in range(0,self.obstacle_num)]

        # for i in range(0,7):
            
        #     #  selector_index 7x10
        #     #  selected_idx 10,
        #     selected_idx=selector_index[i,:]

        #     selected_dist=self.dist_matrix[i,selected_idx]

        #     dist_filter=np.argwhere(selected_dist<=0.05).reshape(-1)
        
        #     if dist_filter.size:

        #         selected_idx = selected_idx[dist_filter]

        #         selector[i] = self.obs_spherecenter[selected_idx,:]

        # for i in range(0,7):

        #     # 先看是不是空list
        #     if selector[i]:

        #         # selected_mat 10*3
        #         selected_mat=self.obs_spherecenter[selected_idx,:]

        #         selected_mat=np.argwhere(selected_mat<self.activate_threshold)

        #         selector[i] = self.obs_spherecenter[:,3]

        # self.joints_force=np.zeros(7,3)

        # for i in range(0,7):

        #     # self.joints_xyz[i] with size 3x1
        #     # selector[i] with size nx3 e.g. 10x3

        #     vec = self.joints_xyz[i]-selector[i]

        #     vec = np.sum(vec,axis=0)

        #     vec = np.linalg.norm(vec)

        #     vec = 

    def UpdateRepForce(self):

        self.rr = 0.1

        idx_min = np.argmin(self.dist_matrix,axis=1)
        dist_min = np.min(self.dist_matrix,axis=1)

        dq_matrix = np.zeros(shape=(7,7))

        for i in range(0,7):
            
            selected_idx = idx_min[i]
            selected_dist = dist_min[i]

            if selected_dist < 0.05:

                vec_diff =self.obs_spherecenter[selected_idx,:]-self.joints_xyz[i,:]

                force_vec = np.linalg.norm(vec_diff)

                force_vec =  vec_diff / force_vec

                force_amp = self.k_rep*(1.0/selected_dist-1.0/self.rr)

                force_rep = force_vec*force_amp

            else:

                force_rep = np.array([0,0,0])

            dx_tmp = np.zeros(shape=(6,1))

            dx_tmp[:3] = force_rep.reshape(3,1)

            # dx_tmp -> size : 6x1
            # dq_tmp -> size : 7x6 * 6x1 = 7x1

            dq_tmp = np.matmul(self.joints_jacobInv[i],dx_tmp)

            # each row is for one joint 

            dq_matrix[i,:] = dq_tmp.reshape(1,7)


        ## sum of dq based on repulsive forece

        # sum along column

        # dq_repForce -> size : 1 * 7
        dq_repForce = np.sum(dq_matrix,axis=0)

        dq_repForce = dq_repForce

        self.rep_force = dq_repForce


    def UpdateResForce(self):
        
        res_force = self.rep_force + self.att_force

        return res_force.reshape(7,)

    def DistanceCalculate(self,X,Y):

        # X [m,k]  Y [n,k]
        # return res[m,n]  
        # element res[i,j] is distance between X[i] and Y[j]

        vector_diff = X[:,None,:] - Y
        dist = np.sqrt(np.einsum('ijk,ijk->ij',vector_diff,vector_diff))

        return dist
    
    def IsExistSphereXML(self):
        pass