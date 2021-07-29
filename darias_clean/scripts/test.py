import sys
import os

dir=os.path.dirname(__file__)
rootlib=os.path.abspath(os.path.join(dir,'..'))
sys.path.append(rootlib)

from Darias.kinematic import Kinematic
from Darias.controller import ControllSpeed_eef
from Darias.utils import DistHomoMatrix

import numpy as np
from copy import deepcopy

class simualtion():

    def __init__(self):

        self.robot=Kinematic()

        self.ParameterConfig()

        self.InitTarget()

    def ParameterConfig(self):

        self.jointidx_eef=7
        self.dt=0.01

        self.enable_pybullet=False

        self.iteration_success=False
        self.iteration_max=1000

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
        
        for i in range(0,self.iteration_max):

            x=self.robot.GetJointState(self.jointidx_eef)

            err=DistHomoMatrix(x,self.T_target_world)

            print("{:*^30}".format('err'))
            print(err)

            # print("{:*^30}".format('x'))
            # print(x)

            dx=ControllSpeed_eef(self.T_target_world,x)

            # print("{:*^30}".format('dx'))
            # print(dx)


            dq=np.matmul(self.robot.JacobWorldInv_eef(),dx)

            # print("{:*^30}".format('dq'))
            # print(dq)


            q=q+dq*self.dt

            self.SimulationStep(q)

def main():

    sim=simualtion()

    sim.StartSim()

    


if __name__ == '__main__':
    main()

        