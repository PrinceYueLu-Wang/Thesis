import sys
import os

dir=os.path.dirname(__file__)
rootlib=os.path.abspath(os.path.join(dir,'..'))
sys.path.append(rootlib)

from Darias.kinematic import Kinematic
from Darias.controller import ControllSpeed_eef
from Darias.utils import DistHomoMatrix
from Darias.utils import PlotData

import numpy as np
from copy import deepcopy
from time import time,sleep

class simualtion():

    def __init__(self):

        self.robot=Kinematic()

        self.ParameterConfig()

        self.InitTarget()

    def ParameterConfig(self):

        self.jointidx_eef=7
        self.dt=0.01

        self.enable_pybullet=False

        self.eps=1e-3

        self.iteration_success=False
        self.iteration_max=1000

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

        else:
            self.T_target_world=args

    def StartSim(self):

        q=deepcopy(self.robot.q_init)
        
        for iter in range(0,self.iteration_max):

            x=self.robot.GetJointState(self.jointidx_eef)
            v=self.robot.VelocityWorld_eef()

            v_des=ControllSpeed_eef(self.T_target_world,x)
            
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

    # sim.plot_data.plot_x_EEF_world()
    sim.plot_data.Plot_dq()



    


if __name__ == '__main__':
    main()

        