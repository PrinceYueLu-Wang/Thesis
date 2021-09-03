import sys
import os

import pygame


dir=os.path.dirname(__file__)
rootlib=os.path.abspath(os.path.join(dir,'..'))
sys.path.append(rootlib)

from Darias.kinematic import Kinematic
from Darias.controller import ControllSpeed_eef,LineContSpeed_eef
from Darias.utils import DistHomoMatrix
from Darias.utils import PlotData
from Darias.envs import PyEnv
from Darias.field import Vector3d,apf

import numpy as np
import quaternion

from copy import deepcopy
from time import time,sleep

from tqdm import tqdm


class simualtion():

    def __init__(self):

        self.ParameterConfig()

        self.robot=Kinematic()

        self.FieldInit()

        if self.enable_pybullet:
            self.bullet=PyEnv()            

        # self.InitTarget([0.3,0,1.6])

        # self.InitTarget([0.73183, 0.2339, 1.4109],
        #                 [0.20901,0.68476,-0.22178,0.66199]
        #                 )

        # self.InitTarget([0.75579,  -0.12606,   1.296],
        #                 [0.21194,   0.7567,  -0.18032,  0.59158]
        #                 )

        # self.InitTarget([0.75579,  0,   1.296],[0, 0.7071068,  0,  0.7071068])

    def ParameterConfig(self):

        self.jointidx_eef=7
        self.dt=0.1

        # self.enable_pybullet=True
        self.enable_pybullet=True

        self.eps=3*1e-3

        self.iteration_success=False
        self.iteration_max=1000

        self.iter=0

        self.plot_data=PlotData()

    def SimulationStep(self,q):

        # Pinocchio update  
        #======================================================#
        self.robot.KinUpdate(q)
        
        #Field update
        #======================================================#
        


        #Pybullet update
        #======================================================#
        self.iter+=1

        if (self.iter % 2 == 0) and self.enable_pybullet:
            self.bullet.BulletUpdate(q)

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

            quater=args[1]
            translation=np.array(args[0])

            quat=np.quaternion(quater[3],quater[0],quater[1],quater[2])
            
            rot_mat=quaternion.as_rotation_matrix(quat)

            T=np.eye(4)
            T[0:3,0:3]=rot_mat
            T[0:3,-1]=translation

            self.T_target_world=T

    def TargetFromQuanTrans(self,*args):

        quater=args[1]
        translation=np.array(args[0])

        quat=np.quaternion(quater[3],quater[0],quater[1],quater[2])
        
        rot_mat=quaternion.as_rotation_matrix(quat)

        T=np.eye(4)
        T[0:3,0:3]=rot_mat
        T[0:3,-1]=translation

        # self.T_target_world=T

        return T

    def HomoFromRotTrans(self,args):

        quater=args[1]
        translation=np.array(args[0])

        quat=np.quaternion(quater[3],quater[0],quater[1],quater[2])
        rot_mat=quaternion.as_rotation_matrix(quat)

        T=np.eye(4)
        T[0:3,0:3]=rot_mat
        T[0:3,-1]=translation
        
        return T

    def KinematicInv(self,trans_target,quater=False):  

        # quater -> x,y,z,w

        robot_inv=Kinematic()

        # q_inv_init=robot_inv.NeutralJointState()
        # q=deepcopy(q_inv_init)

        q=deepcopy(robot_inv.q_init)

        iteration_success=False


        if not quater :
            T_target_world=np.eye(4)
            T_target_world[0:3,-1]=trans_target

        else:

            T_target_world=np.eye(4)
            quat=np.quaternion(quater[3],quater[0],quater[1],quater[2])
            #here input w,x,y,z
            
            rot_mat=quaternion.as_rotation_matrix(quat)

            T_target_world[0:3,0:3]=rot_mat
            T_target_world[0:3,-1]=trans_target
            

        for iter in range(0,self.iteration_max):

            x=robot_inv.GetJointState(self.jointidx_eef)
            v=robot_inv.VelocityWorld_eef()

            v_des=ControllSpeed_eef(T_target_world,x)
            
            dq=np.matmul(robot_inv.JacobWorldInv_eef(),v_des)

            q=q+dq*self.dt

            robot_inv.KinUpdate(q)

            err=DistHomoMatrix(x,T_target_world)

            if err < 0.3*1e-3:
                iteration_success=True
                break

            if iter % 100 == 0:
                print("No. {} with error {}:".format(iter,err))
                print("x : {}".format(x[0:3,-1]))

        return iteration_success

    def FieldInit(self):
        #only eef with att
        #==================================================#
        
        self.FieldList=[apf() for x in np.arange(0,6)]

        self.FieldList.append(apf(enableAttractive=True))

        #==================================================#

        # # joint No. 0  - No.6 without Attractive
        
        # self.FieldList=[apf() for x in np.arange(0,7)]

    def ForceFromField(self):

        statelist=self.robot.JointStateListUpdate()

        # every column is for one joint
        ddq_joint=np.zeros(shape=(7,7))

        for idx in np.arange(0,7):

            f = self.FieldList[idx].ForceUpdate(statelist[idx])

            joint_idx=idx+1

            J_inv = self.robot.JacobWorldInv(joint_idx)

            ddq_joint[:,idx]=np.matmul(J_inv,f)

        ddq=ddq_joint.sum(axis=1)

        return ddq

    def MoveToTarget(self,q_previous,trans,rot):

        # self.InitTarget([0.75579,  0,   1.296],[0, 0.7071068,  0,  0.7071068])
        self.InitTarget(trans,rot)

        q=deepcopy(q_previous)
        
        for iter in range(0,self.iteration_max):

            x=self.robot.GetJointState(self.jointidx_eef)
            v=self.robot.VelocityWorld_eef()

            v_des=ControllSpeed_eef(self.T_target_world,x)
            # v_des=LineContSpeed_eef(self.T_target_world,x)
            
            dq=np.matmul(self.robot.JacobWorldInv_eef(),v_des)

            q=q+dq*self.dt

            self.SimulationStep(q)

            # error: euclidian distance between two Frame 
            err=DistHomoMatrix(x,self.T_target_world)
            print("{:*^30}".format('err'))
            
            print(err)

            if err < self.eps:
                self.iteration_success=True
                print("*********************")
                print("task successed!")

                return q
                break
        
        if err > self.eps:
            print("*********************")
            print("task failed!")

        return q
    
    def MoveToTargetJoy(self,q_previous,command):

        print(command)

        q=deepcopy(q_previous)

        trans_previous=self.robot.GetJointState(self.jointidx_eef,return_type="translation")

        rot=[0, 0.7071068,  0,  0.7071068]
        trans=deepcopy(trans_previous)

        if command == "up":
            trans[0]= trans[0]+0.05
        if command == "down":
            trans[0]= trans[0]-0.05
        if command == "left":
            trans[1]= trans[1]-0.05
        if command == "right":
            trans[1]= trans[1]+0.05

        self.InitTarget(trans,rot)
        
        for iter in range(0,self.iteration_max):

            x=self.robot.GetJointState(self.jointidx_eef)

            v=self.robot.VelocityWorld_eef()

            v_des=ControllSpeed_eef(self.T_target_world,x)
            # v_des=LineContSpeed_eef(self.T_target_world,x)
            
            dq=np.matmul(self.robot.JacobWorldInv_eef(),v_des)

            q=q+dq*self.dt

            self.SimulationStep(q)

            # error: euclidian distance between two Frame 
            err=DistHomoMatrix(x,self.T_target_world)
            print("{:*^30}".format('err'))
            
            print(err)

            if err < self.eps:
                self.iteration_success=True
                print("*********************")
                print("task successed!")

                return q
                break
        
        if err > self.eps:
            print("*********************")
            print("task failed!")
            print("reset to the previous state")


            self.SimulationStep(q_previous)

            return q_previous
    
    def StartSim(self):

        q=deepcopy(self.robot.q_init)
        
        for iter in range(0,self.iteration_max):

            x=self.robot.GetJointState(self.jointidx_eef)
            v=self.robot.VelocityWorld_eef()

            v_des=ControllSpeed_eef(self.T_target_world,x)
            # v_des=LineContSpeed_eef(self.T_target_world,x)
            
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
    
    def StartSimAPF(self):

        q=deepcopy(self.robot.q_init)
        dq=np.zeros((7,))
        
        for iter in range(0,self.iteration_max):

            x=self.robot.GetJointState(self.jointidx_eef)
            v=self.robot.VelocityWorld_eef()

            # ddq=self.ForceFromField()

            # dq=dq+self.dt*ddq
            # q=q+dq*self.dt

            dq=self.ForceFromField()

            # q=q+dq*self.dt
            q=q+dq*0.02

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

    def StartSimDualshock(self):

        #Q init
        #======================================#
        q=deepcopy(self.robot.q_init)
        
        #Move to init state
        #======================================#

        # self.InitTarget([0.75579,  0,   1.296],[0, 0.7071068,  0,  0.7071068])

        trans=[0.75579,  0,   1.296]
        rot=[0, 0.7071068,  0,  0.7071068]

        q_previous=self.MoveToTarget(q,trans=trans,rot=rot)

        #JoyStick Init
        #=====================================#
        pygame.init()

        joystick=pygame.joystick.Joystick(0)

        joystick.init()

        clock = pygame.time.Clock()
        color = 0

        running = True

        button_idx=14

        button_dict={'2':"up",
                    "0":"down",
                    "3":"left",
                    "1":"right",
                    "10":"reset"}


        while running:

            for event in pygame.event.get():

                if event.type == pygame.JOYBUTTONDOWN:

                    idx=str(event.button)

                    if button_dict.__contains__(idx):

                        command=button_dict[idx]

                        if command == "up":
                            print("Button UP is Pressed!")

                        elif command == "down":
                            print("Button DOWN is Pressed!")

                        elif command == "left":
                            print("Button LEFT is Pressed!")

                        elif command == "right":
                            print("Button RIGHT is Pressed!")
                        
                        q_previous=self.MoveToTargetJoy(q_previous,command)


                    else:

                        print("not in the list!")
                
def main():

    sim=simualtion()
    #======================================================#
    sim.StartSimDualshock()

    #======================================================#
    # sim.StartSimAPF()


    #======================================================#
    # sim.StartSim()

    # a=sim.robot.GetJointState(sim.jointidx_eef)
    # field=apf(True)
    # field.ForceUpdate(a)

    #======================================================#
    # print(sim.KinematicInv(
    #     trans_target=[0.73183, 0.2339, 1.4109],
    #     quater=[0.20901,0.68476,-0.22178,0.66199]
    #     ))
    #======================================================#
    # possible pose 3 
    # print(sim.KinematicInv(
    #     trans_target=[0.75579,  -0.12606,   1.296],
    #     quater=[0.21194,   0.7567,  -0.18032,  0.59158]
    #     ))  
    #======================================================#
    # possible pose 4 
    # print(sim.KinematicInv(
    #     trans_target=[0.75579,  0.1,   1.296],
    #     quater=[0, 0.7071068,  0+0.1,  0.7071068]
    #     ))  
     #======================================================#
    # sim.StartSim()
    # print(sim.KinematicInv([0.3,0.1,1.5]))
    # for x in np.linspace(0,0.5,10):
    #     for y in np.linspace(0.2,0.2,1):
    #         for z in np.linspace(1.3,1.6,10):
    #             flag=str(sim.KinematicInv([x,y,z]) )
    #             print("x={},y={},z={},res={}".format(x,y,z,flag))
    # sim.plot_data.Plot_x_EEF_world()
     #======================================================#


if __name__ == '__main__':
    main()

        