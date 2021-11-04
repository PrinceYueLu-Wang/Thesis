import os
import sys
import copy
import contextlib
from time import sleep
from os.path import dirname, join, abspath

import pinocchio as pin
import pybullet as p
import pybullet_data

import cv2
import torch
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits import mplot3d

from cep.utils import numpy2torch, torch2numpy
from cep.liegroups.torch import SO3, SE3

class plotData():
    def __init__(self):
        self.q=[]
        self.dq=[]
        self.ddq=[]
        self.x_EEF_world=[]
        self.v_EEF_world=[]
        self.time=[]
        self.idxEEF=7

    def update(self,q=None,dq=None,ddq=None,x_EEF_world=None,v_EEF_world=None,time=None):

        self.q.append(q)
        self.dq.append(dq)
        self.ddq.append(ddq)
        self.x_EEF_world.append(x_EEF_world)
        self.v_EEF_world.append(v_EEF_world)
        self.time.append(time)

    def plot_q(self,allJoints=False):

        fig = plt.figure(1)

        if allJoints==False:
            temp_q=[x[self.idxEEF-1] for x in self.q]
            plt.plot(self.time,temp_q)
            plt.show()
        elif allJoints==True:
            temp_q=[]
            for i in range(0,7):
                temp_q.append([x[i] for x in self.q])
            for i in range(0,7):
                temp_ax=fig.add_subplot(7,1,i+1)
                plt.plot(self.time,temp_q[i])
            plt.show()

    def plot_dq(self,allJoints=False):

        fig = plt.figure(1)

        if allJoints==False:
            temp_dq=[x[self.idxEEF-1] for x in self.dq]
            # print(len(temp_dq))
            # print(len(self.time))
            plt.plot(self.time,temp_dq)
            plt.show()
        elif allJoints==True:
            temp_dq=[]
            for i in range(0,7):
                temp_dq.append([x[i] for x in self.dq])
            for i in range(0,7):
                temp_ax=fig.add_subplot(7,1,i+1)
                temp_ax.set_ylim(-1,1)
                plt.plot(self.time,temp_dq[i])
            plt.show()

    def plot_x_EEF_world(self):
        
        fig = plt.figure(1)
        ax = plt.axes(projection='3d')
        ax.set_xlabel('x')
        ax.set_ylabel('y')
        ax.set_zlabel('z')


        data_x=[x[0] for x in self.x_EEF_world]
        data_y=[x[1] for x in self.x_EEF_world]
        data_z=[x[2] for x in self.x_EEF_world]

        ax.set_xlim(min(data_x),max(data_x))
        ax.set_ylim(min(data_y),max(data_y))
        ax.set_zlim(min(data_z),max(data_z))

        ax.plot3D(data_x, data_y, data_z, 'red')
        ax.scatter3D(data_x, data_y, data_z, c=data_x, cmap='Greens')

        plt.show()

class darias():
    def __init__(self):


        self.JOINT_ID=7
        self.eps = 2*1e-3
        self.IT_MAX = 1000
        self.damp = 1e-12
        self.jointNum=7

        model_dir=dirname(__file__)
        model_absPath=join(model_dir,"model/darias_singleArm.urdf")
        self.model_absPath=model_absPath

        self.model=pin.buildModelFromUrdf(model_absPath) 
        self.data = self.model.createData()
        self.dataRandom=self.model.createData()


        self.jointIdx_pinocchio=[1,2,3,4,5,6,7]
        self.frameIdx_pinocchio=[7,9,11,13,15,17,19]
        self.jointIdx_pybullet=[26,27,28,29,30,31,32]

        self.jointDict_pin2bullet=dict((pin,bullet) for pin,bullet in zip(self.jointIdx_pinocchio,self.jointIdx_pybullet))
        self.jointDict_bullet2pin=dict((bullet,pin) for bullet,pin in zip(self.jointIdx_pybullet,self.jointIdx_pinocchio))
        self.jointDict_pin2frame=dict((pin,frame) for pin,frame in zip(self.jointIdx_pinocchio,self.frameIdx_pinocchio))

        
        self.qrandom=pin.randomConfiguration(self.model)
        
        pin.forwardKinematics(self.model,self.dataRandom,self.qrandom)
        pin.framesForwardKinematics(self.model,self.dataRandom,self.qrandom)

        self.x_randomR=copy.deepcopy(self.dataRandom.oMi[self.JOINT_ID].rotation)
        self.x_randomT=copy.deepcopy(self.dataRandom.oMi[self.JOINT_ID].translation)

        # print(self.x_randomR)
        # print(self.x_randomT)

        self.q_init=pin.neutral(self.model)
        pin.forwardKinematics(self.model,self.data,self.q_init)
        pin.framesForwardKinematics(self.model,self.data,self.q_init)

        self.x_initR=copy.deepcopy(self.data.oMi[self.JOINT_ID].rotation)
        self.x_initT=copy.deepcopy(self.data.oMi[self.JOINT_ID].translation)

        # print(self.x_initR)
        # print(self.x_initT)


        self.plotData=plotData()

    def pybulletInit(self):
        with contextlib.redirect_stdout(None):
            physicsCilent = p.connect(p.GUI)
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        p.setGravity(0, 0, -10)
        planeId = p.loadURDF("plane.urdf")
        startPos = [0, 0, 0]
        startOrientation = p.getQuaternionFromEuler([0, 0, 0])
        self.robotID = p.loadURDF("/home/faye/Desktop/thesis/pinocchio/model/darias_singleArm.urdf",startPos,startOrientation)


        startPos = [0, 0, 0]
        startOrientation = p.getQuaternionFromEuler([0, 0, 0])
        sphere_startPos = [0.5, -0.6, 1.8]
        sphere_startOrientation = p.getQuaternionFromEuler([0, 0, 0])

        SphereId = p.loadURDF("/home/faye/Desktop/thesis/pinocchio/model/sphere_10cm.urdf",sphere_startPos,sphere_startOrientation)

    def se3ToTorch(self, SE3):

        # Transform a SE3 to a  (4, 4) transformation matrix.

        r = numpy2torch(SE3.rotation)
        t = numpy2torch(SE3.translation)
        x1 = torch.cat((r, t.reshape(3, 1)), 1)
        homo = torch.tensor([[0, 0, 0, 1]])
        Tf = torch.cat((x1, homo), 0)

        return Tf

    def motionToTorch(self,Motion):

        v=numpy2torch(Motion.linear)
        w=numpy2torch(Motion.angular)

        tmp=torch.cat((v,w),dim=0)

        tmp.reshape(6,1)

        return tmp

    def velocityControl(self,target,state):
        '''
        
            target 4x4 homo Trans Matrix  in format torch.4x4

            state 4x4 homo trans matrix   in format torch.4x4
        '''

        T_worldMtarget=target 
        T_worldMobj=state

        T_targetMobj=torch.matmul(torch.inverse(T_worldMtarget),T_worldMobj)

        Xe = SE3.from_matrix(T_targetMobj, normalize=True) 

        xtl = Xe.log()  
        vtl = -xtl

        A = SE3.from_matrix(T_worldMtarget)
        Adj_lw = A.adjoint()

        ve_world = torch.matmul(Adj_lw, vtl)

        return ve_world

    def accelerationControl(self,target,state,velocity):
        '''
            target 4x4 homo Trans Matrix in format torch.4x4

            state 4x4 homo trans matrix in format torch.4x4

            velocity 6x1 matrix motion in format torch.6x1
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


        scale = 20
        mu = scale * ve_world - 1.2 * scale * velocity

        return mu
      
    def startSimulation(self):

        self.errorPrintFlag=True
        self.enalbe_animation=False
        self.controlFlag=1
        # TODO: controlFlag -> 1  acc control
        # TODO: controlFlag -> 0  velocity control

        if self.enalbe_animation:
            self.pybulletInit()
        
        self.controlMode="acc" if self.controlFlag==1 else "velocity"

        self.dt=0.001
        
        q=self.q_init

        # TODO: T_world2target in format Torch
        SE3_world2target=pin.SE3(self.x_randomR,self.x_randomT)
        T_world2target=self.se3ToTorch(SE3_world2target)

        # TODO: T_EEF in format Torch
        SE3_EEF=self.data.oMi[self.JOINT_ID]
        T_EEF=self.se3ToTorch(SE3_EEF)

        taskSuccess=False

        for iter in range(0,1000):

            # TODO : error euclidian distance between target and obj
            SE3_obj2tar=SE3_world2target.actInv(SE3_EEF)
            error = np.linalg.norm(pin.log(SE3_obj2tar).vector)


            if error < self.eps:
                taskSuccess= True
                break

            pin.computeJointJacobians(self.model, self.data, q)
            pin.updateFramePlacements(self.model, self.data)


            # TODO : v_EEF_world in format torch
            v_EEF_world=pin.getFrameVelocity(self.model,self.data,
                        self.jointDict_pin2frame[self.JOINT_ID],pin.ReferenceFrame.WORLD)
            v_EEF_world=self.motionToTorch(v_EEF_world)

            # TODO : J_pesudo Matrix in format torch: size 6x6

            J = pin.getFrameJacobian(self.model,self.data,
                        self.jointDict_pin2frame[self.JOINT_ID],
                        pin.ReferenceFrame.WORLD) 
            J = numpy2torch(J)
            J_pesudo = torch.matmul(J.T,torch.inverse(torch.matmul(J,J.T)+1e-6*torch.eye(6)))

            if self.controlMode =="acc":

                ve_w=self.velocityControl(state=T_EEF,target=T_world2target)
                mu=self.accelerationControl(state=T_EEF,target=T_world2target,velocity=v_EEF_world)

                ddq=torch.matmul(J_pesudo,mu)
                ddq=torch2numpy(ddq)
                dq=torch.matmul(J_pesudo,ve_w)
                dq=torch2numpy(dq)

                dq=dq+ddq*self.dt
                q=q+dq*self.dt

                self.plotData.update(q,dq,ddq,)

            elif self.controlMode =="velocity":

                ve_w=self.velocityControl(state=T_EEF,target=T_world2target)
                
                dq=torch.matmul(J_pesudo,ve_w)
                ddq=torch.zeros(self.jointNum,)

                dq=dq+ddq*self.dt
                dq = torch2numpy(dq)
                q = q + dq * self.dt

            
            if iter % 20==0 and self.enalbe_animation:
                for (pino,bullet) in zip(self.jointIdx_pinocchio,self.jointIdx_pybullet):
                    p.resetJointState(self.robotID, jointIndex=bullet, targetValue=q[pino-1])
                    sleep(0.1)
                p.stepSimulation()

            pin.forwardKinematics(self.model, self.data, q)
            pin.framesForwardKinematics(self.model, self.data, q)
            
            if self.errorPrintFlag:
                print("NO.{} iter shows error {}".format(iter,error))

            x_EEF_world=copy.deepcopy(self.data.oMi[self.JOINT_ID].translation)

            self.plotData.update(q=q,dq=dq,ddq=ddq,
                    x_EEF_world=x_EEF_world,v_EEF_world=copy.deepcopy(v_EEF_world),
                    time=self.dt*iter)
            # print(x_EEF_world)


def main():
    sim=darias()
    sim.startSimulation()
    sim.plotData.plot_dq(allJoints=True)
    # sim.plotData.plot_x_EEF_world()
    # 

if __name__=="__main__":
    main()
