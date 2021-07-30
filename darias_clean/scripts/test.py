import sys
import os

dir=os.path.dirname(__file__)
rootlib=os.path.abspath(os.path.join(dir,'..'))
sys.path.append(rootlib)

from Darias.kinematic import Kinematic
from Darias.controller import ControllSpeed_eef,LineContSpeed_eef
from Darias.utils import DistHomoMatrix
from Darias.utils import PlotData

import numpy as np
from copy import deepcopy
from time import time,sleep

from tqdm import tqdm

class simualtion():

    def __init__(self):

        self.robot=Kinematic()

        self.ParameterConfig()

        self.InitTarget([0.4,0.4,1.6])

    def ParameterConfig(self):

        self.jointidx_eef=7
        self.dt=0.01

        self.enable_pybullet=False

        self.eps=1e-3

        self.iteration_success=False
        self.iteration_max=2000

        self.plot_data=PlotData()

    def SimulationStep(self,q,iteration=1):

        self.robot.KinUpdate(q)

        if (iteration % 20 == 0) and self.enable_pybullet:
            pass

    def InitTarget(self,*args):

        if not args:
            
            rot = np.eye(3)
            translation=np.array([0.3,0.0,1.4])

            T=np.eye(4)
            T[0:3,0:3]=rot
            T[0:3,-1]=translation

            self.T_target_world=T

        elif len(args) == 1:
            rot = np.eye(3)
            translation=np.array(args)

            T=np.eye(4)
            T[0:3,0:3]=rot
            T[0:3,-1]=translation

            self.T_target_world=T
        
        elif len(args) == 2:

            rot = np.array(args[0])
            translation=np.array(args[1])

            T=np.eye(4)
            T[0:3,0:3]=rot
            T[0:3,-1]=translation

            self.T_target_world=T

    def KinematicInv(self,trans_target):  

        robot_inv=Kinematic()

        q=robot_inv.NeutralJointState()

        iteration_success=False

        T_target_world=np.eye(4)
        T_target_world[0:3,-1]=trans_target

        for iter in range(0,self.iteration_max):

            x=robot_inv.GetJointState(self.jointidx_eef)
            v=robot_inv.VelocityWorld_eef()

            v_des=ControllSpeed_eef(T_target_world,x)
            
            dq=np.matmul(robot_inv.JacobWorldInv_eef(),v_des)

            q=q+dq*self.dt

            robot_inv.KinUpdate(q)

            err=DistHomoMatrix(x,self.T_target_world)

            if err < self.eps:
                iteration_success=True
                break

            err=DistHomoMatrix(x,self.T_target_world)
            # if iter % 100 == 0:
            #     # print("{:*^30}".format('err'))
            #     print("No. {} with error {}:".format(iter,err))
        
        # if iteration_success:
        #     # print("{:*^30}".format('q'))
        #     # print(q)
        #     return x
        # else:
        #     return -1

        return iteration_success

    def StartSim(self):

        q=deepcopy(self.robot.q_init)
        
        for iter in range(0,self.iteration_max):

            x=self.robot.GetJointState(self.jointidx_eef)
            v=self.robot.VelocityWorld_eef()

            v_des=ControllSpeed_eef(self.T_target_world,x)
            v_des=LineContSpeed_eef(self.T_target_world,x)
            
            dq=np.matmul(self.robot.JacobWorldInv_eef(),v_des)

            q=q+dq*self.dt

            self.SimulationStep(q)
            self.plot_data.DataUpdate(
                q=q,dq=dq,
                ddq=None,
                x_EEF_world=x,
                v_EEF_world=v,
                time=iter*self.dt)

            # error: euclidian distance between two Frame 
            err=DistHomoMatrix(x,self.T_target_world)
            print("{:*^30}".format('err'))
            print(err)

            if err < self.eps:
                self.iteration_success=True
                break

def main():

    sim=simualtion()

    sim.StartSim()
    # print(sim.KinematicInv([0.3,0.0,1.4]))
    sim.plot_data.Plot_x_EEF_world()
    # sim.plot_data.Plot_q()

    # range_list=[]

    # progressbar1=tqdm(total=10,leave=False,desc="Loop1",position=1)
    # progressbar2=tqdm(total=10,leave=True,desc="Loop2",position=2)

    # for x in np.arange(0.0,1.0,0.1):
    #     for z in np.arange(1.0,2.0,0.1):

    #         flag=sim.KinematicInv([x,0.0,z])

    #         if flag:
    #             range_list.append([x,z])

    #         progressbar2.update(1)

    #     progressbar2.reset()
    #     progressbar1.update(1)

    # progressbar1.close()
    # progressbar2.close()



    # for item in range_list:
    #     print("x : {} , z : {}".format(item[0],item[1]))


if __name__ == '__main__':
    main()

        