import os
import sys
import pinocchio as pin
import cv2
import torch
from os.path import dirname, join, abspath
import numpy as np
import pybullet as p
import pybullet_data
from time import sleep
from cep.utils import numpy2torch, torch2numpy
from cep.liegroups.torch import SO3, SE3



class darias():
    def __init__(self):


        self.JOINT_ID=7
        self.eps = 1e-3
        self.IT_MAX = 1000
        self.DT = 1e-1
        self.damp = 1e-12

        model_dir=dirname(__file__)
        model_absPath=join(model_dir,"model/darias.urdf")
        self.model_absPath=model_absPath

        self.model=pin.buildModelFromUrdf(model_absPath) 
        self.data = self.model.createData()

        self.q_init=pin.neutral(self.model)
        self.x_init=self.data.oMi[7].translation

        pin.forwardKinematics(self.model,self.data,self.q_init)
        pin.framesForwardKinematics(self.model,self.data,self.q_init)

        self.x_initPose=self.data.oMi[self.JOINT_ID].rotation
        self.x_initTrans=self.data.oMi[self.JOINT_ID].translation

        # physicsCilent = p.connect(p.GUI)
        # # 添加资源路径
        # p.setAdditionalSearchPath(pybullet_data.getDataPath())
        # # 设置环境重力加速度
        # p.setGravity(0, 0, -10)
        # # 加载URDF模型，此处是加载蓝白相间的陆地
        # planeId = p.loadURDF("plane.urdf")
        # # 加载机器人，并设置加载的机器人的位姿
        # startPos = [0, 0, 0]
        # startOrientation = p.getQuaternionFromEuler([0, 0, 0])
        # self.robotID = p.loadURDF("/home/faye/Desktop/thesis/pinocchio/model/darias_singleArm.urdf",startPos,startOrientation)


        # startPos = [0, 0, 0]
        # startOrientation = p.getQuaternionFromEuler([0, 0, 0])
        # sphere_startPos = [0.5, -0.6, 1.8]
        # sphere_startOrientation = p.getQuaternionFromEuler([0, 0, 0])

        # SphereId = p.loadURDF("/home/faye/Desktop/thesis/pinocchio/model/sphere_10cm.urdf",sphere_startPos,sphere_startOrientation)
    def se3ToTransfrom(self, SE3):

        # Transform a SE3 to a  (4, 4) transformation matrix.

        r = numpy2torch(SE3.rotation)
        t = numpy2torch(SE3.translation)
        x1 = torch.cat((r, t.reshape(3, 1)), 1)
        homo = torch.tensor([[0, 0, 0, 1]])
        Tf = torch.cat((x1, homo), 0)

        return Tf            

    def calculation(self,x):

        q=self.q_init
        
        oMdes=pin.SE3(self.q_initPose,np.array(x))

        i = 0

        while True:
            pin.forwardKinematics(self.model,self.data,q)

            dMi = oMdes.actInv(self.data.oMi[self.JOINT_ID])
            err = pin.log(dMi).vector

            if np.linalg.norm(err) < self.eps:
                success = True
                break
            if i >= self.IT_MAX:
                success = False
                break

            J = pin.computeJointJacobian(self.model, self.data, q, self.JOINT_ID)
            v = - J.T.dot(np.linalg.solve(J.dot(J.T) + self.damp * np.eye(6), err))
            q = pin.integrate(self.model, q, v*self.DT)

            i += 1

        return q,success

    def velocity(self,target,state):

        T_target2world=target
        T_obj2world=state

        T_obj2tar=torch.matmul(torch.inverse(T_target2world),T_obj2world)

        Xe = SE3.from_matrix(T_obj2tar, normalize=True) 

        xtl = Xe.log()  
        vtl = -xtl

        A = SE3.from_matrix(T_target2world)
        Adj_lw = A.adjoint()
        ve_w = torch.matmul(Adj_lw, vtl)

    def startSimulation(self,x_des=[0.5,0.5,1.3],iterSeg=50):

        # x_init=self.q_initTrans[0]
        # y_init=self.q_initTrans[1]
        # z_init=self.q_initTrans[2]

        # pointList_x=[ x_des[0]+(x_des[0]-x_init)/iterSeg*i for i in range(0,iterSeg+1)]
        # pointList_y=[ x_des[1]+(x_des[1]-y_init)/iterSeg*i for i in range(0,iterSeg+1)]
        # pointList_z=[ x_des[2]+(x_des[2]-z_init)/iterSeg*i for i in range(0,iterSeg+1)]

        # pointList=[[x,y,z] for x,y,z in zip(pointList_x,pointList_y,pointList_z)]

        # # trajectoryList_q=[]

        # flag=True

        # x_cur=self.x_init

        # while(flag):

        #     x_diff=x_des-x_cur

        #     J=pin.getJointJacobian(self.model,self.data,self.JOINT_ID,reference_frame=pin.ReferenceFrame.WORLD)

        #     J = numpy2torch(J)


        q=self.q_init
        
        # oMdes=pin.SE3(np.eye(3),np.array([0.5,0.7,1.2]))

        # i = 0

        # while True:
        #     pin.forwardKinematics(self.model,self.data,q)

        #     dMi = oMdes.actInv(self.data.oMi[self.JOINT_ID])
        #     err = pin.log(dMi).vector

        #     if np.linalg.norm(err) < self.eps:
        #         success = True
        #         break
        #     if i >= self.IT_MAX:
        #         success = False
        #         break

        #     J = pin.computeJointJacobian(self.model, self.data, q, self.JOINT_ID)
        #     v = - J.T.dot(np.linalg.solve(J.dot(J.T) + self.damp * np.eye(6), err))
        #     q = pin.integrate(self.model, q, v*self.DT)

        #     for counter in range(1,8):
        #         p.resetJointState(self.robotID,jointIndex=counter+25,targetValue=q[counter])
        #     i=i+1
        #     p.stepSimulation()
        #     sleep(0.5)

        



        # for i in range(0,len(pointList)):

        #     q_tmp,flag=self.calculation(x=pointList[i])

        #     print("No.{} Seg is successful : {}".format(i,flag))

        
        #     trajectoryList_q.append(q_tmp)

        # simCounter=0
        
        # for q in trajectoryList_q:

        #     for jointIdx in range(1,8):

        #         p.resetJointState(self.robotID,jointIndex=jointIdx+25,targetValue=q[jointIdx])

        #     p.stepSimulation()
        #     sleep(0.5)
        #     p.addUserDebugText("No.{} Seg is ".format(simCounter), textColorRGB=[0, 1, 0],
        #     textSize=5,textPosition=[0, 0, 0],lifeTime=0.5)
        #     simCounter = simCounter + 1.
        #     print(simCounter)
            
        # for i in range(0,len(pointList)):
        #     print(trajectoryList_q[i][7])
        # print(trajectoryList_q[0])

        # for i in range(0,100):
        #     p.resetJointState(self.robotID,jointIndex=27,targetValue=2/100*i)
        #     p.stepSimulation()
        #     sleep(0.5)
        #     p.addUserDebugText("angel is {} ".format(2/100*i), textColorRGB=[0, 1, 0],
        #     textSize=5,textPosition=[0, 0, 0],lifeTime=0.5)

        # names=[]
        # idx=[]
        # for i in range(0,43):
        #     tmp=p.getJointInfo(self.robotID,jointIndex=i)
        #     names.append(tmp[1])
        #     print("name {} ,idx {}".format(tmp[0],tmp[1]))

        
def main():
    sim=darias()
    sim.startSimulation()

if __name__=="__main__":
    main()


