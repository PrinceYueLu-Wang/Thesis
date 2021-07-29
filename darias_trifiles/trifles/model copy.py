import os
import sys
import pinocchio as pin
import cv2
import torch
from os.path import dirname, join, abspath
import numpy as np
import pybullet as pw
import copy
import pybullet_data
from time import sleep
from cep.utils import numpy2torch, torch2numpy
from cep.liegroups.torch import SO3, SE3



class darias():
    def __init__(self):


        self.JOINT_ID=7
        self.eps = 1e-2
        self.IT_MAX = 1000
        self.DT = 1e-1
        self.damp = 1e-12

        

        model_dir=dirname(__file__)
        model_absPath=join(model_dir,"model/darias_singleArm.urdf")
        self.model_absPath=model_absPath

        self.model=pin.buildModelFromUrdf(model_absPath) 
        self.data = self.model.createData()


        self.jointIdx_pinocchio=[1,2,3,4,5,6,7]
        self.frameIdx_pinocchio=[7,9,11,13,15,17,19]
        self.jointIdx_pybullet=[26,27,28,29,30,31,32]

        self.jointDict_pin2bullet=dict((pin,bullet) for pin,bullet in zip(self.jointIdx_pinocchio,self.jointIdx_pybullet))
        self.jointDict_bullet2pin=dict((bullet,pin) for bullet,pin in zip(self.jointIdx_pybullet,self.jointIdx_pinocchio))


        self.qrandom=pin.randomConfiguration(self.model)
        pin.forwardKinematics(self.model,self.data,self.qrandom)
        pin.framesForwardKinematics(self.model,self.data,self.qrandom)
        self.x_randomR=copy.deepcopy(self.data.oMi[self.JOINT_ID].rotation)
        self.x_randomT=copy.deepcopy(self.data.oMi[self.JOINT_ID].translation)

        self.q_init=pin.neutral(self.model)
        pin.forwardKinematics(self.model,self.data,self.q_init)
        pin.framesForwardKinematics(self.model,self.data,self.q_init)
        
        # self.x_initPose=self.data.oMi[self.JOINT_ID].rotation
        # self.x_initTrans=self.data.oMi[self.JOINT_ID].translation

        self.pybulletInit()
        
        # print(self.x_randomR)
        # print(self.x_randomT)
        # print(self.x_initPose)
        # print(self.x_initTrans)


    def pybulletInit(self):
        physicsCilent = p.connect(p.GUI)
        # 添加资源路径
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        # 设置环境重力加速度
        p.setGravity(0, 0, -10)
        # 加载URDF模型，此处是加载蓝白相间的陆地
        planeId = p.loadURDF("plane.urdf")
        # 加载机器人，并设置加载的机器人的位姿
        startPos = [0, 0, 0]
        startOrientation = p.getQuaternionFromEuler([0, 0, 0])
        self.robotID = p.loadURDF("/home/faye/Desktop/thesis/pinocchio/model/darias_singleArm.urdf",startPos,startOrientation)


        startPos = [0, 0, 0]
        startOrientation = p.getQuaternionFromEuler([0, 0, 0])
        sphere_startPos = [0.5, -0.6, 1.8]
        sphere_startOrientation = p.getQuaternionFromEuler([0, 0, 0])

        SphereId = p.loadURDF("/home/faye/Desktop/thesis/pinocchio/model/sphere_10cm.urdf",sphere_startPos,sphere_startOrientation)

    def se3ToTransform(self, SE3):

        # Transform a SE3 to a  (4, 4) transformation matrix.

        r = numpy2torch(SE3.rotation)
        t = numpy2torch(SE3.translation)
        x1 = torch.cat((r, t.reshape(3, 1)), 1)
        homo = torch.tensor([[0, 0, 0, 1]])
        Tf = torch.cat((x1, homo), 0)

        return Tf


    def velocity(self,target,state):
        '''
            target 4x4 homo Trans Matrix

            state 4x4 homo trans matrix
        '''

        T_target2world=target
        T_obj2world=state

        T_obj2target=torch.matmul(torch.inverse(T_target2world),T_obj2world)

        Xe = SE3.from_matrix(T_obj2target, normalize=True) 

        xtl = Xe.log()  
        vtl = -xtl

        A = SE3.from_matrix(T_target2world)
        Adj_lw = A.adjoint()

        ve_world = torch.matmul(Adj_lw, vtl)

        return ve_world

    def startSimulation(self,x_des=[0.5,0.5,1.3],iterSeg=50):

        q=self.q_init

        # SE3_world2target=pin.SE3(np.eye(3),np.array([0.5,0.5,1.3]))
        SE3_world2target=pin.SE3(self.x_randomR,self.x_randomT)

        T_world2target=self.se3ToTransform(SE3_world2target)

        SE3_EEF=self.data.oMi[self.JOINT_ID]

        T_EEF=self.se3ToTransform(SE3_EEF)

        dt=0.01

        flag=False

  

        for iter in range(0,1000):

            SE3_obj2tar=SE3_world2target.actInv(SE3_EEF)

            error = np.linalg.norm(pin.log(SE3_obj2tar).vector)

        
            if error < self.eps:
                flag= True
                break

            pin.computeJointJacobians(self.model, self.data, q)
            pin.updateFramePlacements(self.model, self.data)

            #J = pin.getFrameJacobian(self.model, self.data,self.JOINT_ID,ReferenceFrame=pin.ReferenceFrame.WORLD) 
            J = pin.getFrameJacobian(self.model,self.data,19,pin.ReferenceFrame.WORLD) 

            J = numpy2torch(J)


            ve_w=self.velocity(state=T_EEF,target=T_world2target)

            
            #J_pesudo = J.T.dot(np.linalg.solve(J.dot(J.T) + self.damp * torch.eye(6), ve_w))
            J_pesudo = torch.matmul(J.T,torch.inverse(torch.matmul(J,J.T)+1e-12*torch.eye(6)))

            dq=torch.matmul(J_pesudo,ve_w)

            dq = torch2numpy(dq)

            dt = 0.001
            q = q + dq * dt

            
            print("NO.{} iter shows error {}".format(iter,error))

            if iter % 20==0:
                for (pino,bullet) in zip(self.jointIdx_pinocchio,self.jointIdx_pybullet):
                    p.resetJointState(self.robotID, jointIndex=bullet, targetValue=q[pino-1])
                    sleep(0.1)
                p.stepSimulation()


            pin.forwardKinematics(self.model, self.data, q)
            pin.framesForwardKinematics(self.model, self.data, q)



def main():
    sim=darias()
    sim.startSimulation()

if __name__=="__main__":
    main()


