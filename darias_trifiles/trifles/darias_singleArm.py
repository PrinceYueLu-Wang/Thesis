import pinocchio as pin
import numpy as np
import pybullet as p
import pybullet_data
import torch
from torch import tensor
from pinocchio.robot_wrapper import RobotWrapper
import os
from copy import deepcopy
import matplotlib.pyplot as plt
import time
from cep.utils import numpy2torch, torch2numpy
from cep.liegroups.torch import SO3, SE3
import cv2


class darias_OSC:
    def __init__(self):
        
        self.loadModel()
        self.initPybullt()


    def loadModel(self):

        # print("function {} is running!".format(""))
        print("-"*10)
        print("function {} is running!".format("loadModel"))

        self.scriptDir=os.path.dirname(__file__)
        self.modelPath=os.path.join(self.scriptDir,"model/darias_singleArm.urdf")

        self.robot=pin.buildModelFromUrdf(self.modelPath)

        print("model is loaded!")
        print("model has name :{}".format(self.robot.name))

        self.robotData=self.robot.createData()

        q_init=pin.randomConfiguration(self.robot)

        pin.forwardKinematics(self.robot,self.robotData,q_init)

        self.nq=self.robot.nq

        self.armJointIdx_right_pinocchio=[1,2,3,4,5,6,7]
        self.armJointIdx_right_pybullet=[26,27,28,29,30,31,32]
        self.armJointIdx_left_pinocchio=[1,2,3,4,5,6,7]
        self.armJointIdx_left_pybullet=[26,27,28,29,30,31,32]


    def calculate_vtl(self,curState_EE,tarState_EE):

        # print("-"*10)
        # print("function {} is running!".format("calculate_vtl"))

        x = curState_EE[0]  # Tensor(4, 4), end-effector rotation and position SE(3)
        #v = state[1]  # Tensor (1, 6), end-effector spatial velocity V_b

        R_inv = torch.inverse(tarState_EE)
        Htl = torch.matmul(R_inv, x)  # R_inv * X
        Xe = SE3.from_matrix(Htl, normalize=True)  # <cep.liegroups.torch.se3.SE3Matrix>, SE(3)
        xtl = Xe.log()  # Tensor(1, 6), (omega, V)
        vtl = -xtl

        A = SE3.from_matrix(tarState_EE)
        Adj_lw = A.adjoint()
        ve_w = torch.matmul(Adj_lw, vtl)

        return ve_w


    def calculate_mu(self,curState_EE,tarState_EE):
        # print("-"*10)
        # print("function {} is running!".format("calculate_mu"))
        x = curState_EE[0]  # Tensor(4, 4), end-effector rotation and position SE(3)
        v = curState_EE[1]  # Tensor (1, 6), end-effector spatial velocity V_b
        # index = [3, 4, 5, 0, 1, 2]
        # v = v[index]

        R_inv = torch.inverse(tarState_EE)
        Htl = torch.matmul(R_inv, x)  # R_inv * X
        Xe = SE3.from_matrix(Htl, normalize=True)  # <cep.liegroups.torch.se3.SE3Matrix>, SE(3)
        xtl = Xe.log()  # Tensor(1, 6), (omega, V)
        vtl = -xtl

        A = SE3.from_matrix(tarState_EE)
        Adj_lw = A.adjoint()
        ve_w = torch.matmul(Adj_lw, vtl)

        # TODO: Acceleration control
        scale = 20
        mu = scale * ve_w - 1.2 * scale * v

        return mu



    def initPybullt(self):

        physicsClient = p.connect(p.GUI)

        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        p.resetSimulation(p.RESET_USE_DEFORMABLE_WORLD)
        p.setGravity(0, 0, -9.81)


        planeId = p.loadURDF("plane.urdf")   


        startPos = [0., 0., 0.]
        startOrientation = [0., 0., 0.]

        robotId = p.loadURDF(self.modelPath, startPos, p.getQuaternionFromEuler([0., 0., 0.]), useFixedBase=1)

        p.loadURDF('./model/sphere_10cm.urdf', np.array([0.8, 0., 0.8]), # TODO: Put an object in target postion
                   p.getQuaternionFromEuler([0, 0, 0]),
                   useFixedBase=True)
        
    
        self.jointIdx_EE=1
        self.frameIdx_EE=19

        self.transInit_EE=self.robotData.oMi[1].translation
        self.poseInit_EE=self.robotData.oMi[1].rotation
        self.stateInit_EE=pin.SE3(self.poseInit_EE,self.transInit_EE)

        self.transTarget_EE=np.matrix([0.5,0.5,0.5])
        self.poseTarget_EE=self.poseInit_EE.copy()
        self.stateTarget_EE=pin.SE3(self.transTarget_EE,self.poseTarget_EE)
        

        jointIdx_pinocchio=[1,2,3,4,5,6,7]
        jointIdx_pybullet=[26,27,28,29,30,31,32]

        iteration=100

        for i in range(0,iteration):

            q_target=np.zeros(self.nq)
            q_random=pin.randomConfiguration(self.robot)

            for dictidx,jointidx in enumerate(self.armJointIdx_left):
                jointidx=jointidx-1

                print()

                q_target[jointidx]=q_random[jointidx]

                p.resetJointState(robotId, jointidx, q_target[jointidx])

            p.stepSimulation()
             
            time.sleep(2)
        
        p.disconnect(p.GUI)

        

        

    

if __name__=="__main__":
    print("start simulation")

    T=darias_OSC()





