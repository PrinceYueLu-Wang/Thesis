import os
import pinocchio as pin
import numpy as np
from copy import deepcopy



class Kinematic():

    def __init__(self):

        self.UrdfLoading()

        self.JointInfoConfig()

        self.parameterConfig()

        self.ModelInit()

    def parameterConfig(self):
        self.DAMP=1e-4

    def UrdfLoading(self):

        self.filePath=os.path.dirname(__file__)

        self.robotFolder=os.path.join(self.filePath,'../model')
        self.robotname="darias_singleArm.urdf"
        self.robotPath=os.path.join(self.robotFolder,self.robotname)

        self.model=pin.buildModelFromUrdf(self.robotPath) 
        self.data = self.model.createData()

    def JointInfoConfig(self):

        self.jointid_eef=7
        self.frameid_eef=19

        self.joint_num=7

        self.jointIdx_pinocchio=[1,2,3,4,5,6,7]
        self.frameIdx_pinocchio=[7,9,11,13,15,17,19]
        # self.jointIdx_pybullet=[26,27,28,29,30,31,32]

        # self.jointDict_pin2bullet=dict((pin,bullet) for pin,bullet in zip(self.jointIdx_pinocchio,self.jointIdx_pybullet))
        # self.jointDict_bullet2pin=dict((bullet,pin) for bullet,pin in zip(self.jointIdx_pybullet,self.jointIdx_pinocchio))
        
        self.jointDict_pin2frame=dict((pin,frame) for pin,frame in zip(self.jointIdx_pinocchio,self.frameIdx_pinocchio))

    def ModelInit(self,*args):
        if not args:
            q=pin.randomConfiguration(self.model)

            self.q_init=deepcopy(q)

            self.KinUpdate(q)
        else :
            self.q_init=deepcopy(q)

            self.KinUpdate(args)
        
    def GetFrameId(self,joint_idx):

        return self.jointDict_pin2frame[joint_idx]

    def JacobWorld(self,joint_idx):

        return  pin.getFrameJacobian(
                  self.model,
                  self.data,
                  self.GetFrameId(joint_idx),
                  pin.ReferenceFrame.WORLD)
        
    def JacobWorld_eef(self):

        return  pin.getFrameJacobian(
                  self.model,
                  self.data,
                  self.frameid_eef,
                  pin.ReferenceFrame.WORLD)

    def JacobWorldInv_eef(self):

        J=self.JacobWorld_eef()

        return np.matmul(J.T,
             np.linalg.inv(
             np.matmul(J,J.T)
            +self.DAMP*np.eye(6)) )

    def JacobWorldInv(self,joint_idx):

        J=self.JacobWorld(joint_idx)

        return np.matmul(
               J.T,
               np.linalg.inv(
                  np.matmul(J,J.T)
                  +self.DAMP*np.eye(6)
                  ) 
               )

    def VelocityWorld(self,joint_idx):

        return  pin.getFrameVelocity(
                  self.model,
                  self.data,
                  self.GetFrameId(joint_idx),
                  pin.ReferenceFrame.WORLD
        )

    def VelocityWorld_eef(self):

        return  pin.getFrameVelocity(
                  self.model,
                  self.data,
                  self.frameid_eef,
                  pin.ReferenceFrame.WORLD)

    def KinUpdate(self,q):
        
        pin.computeJointJacobians(self.model, self.data, q)
        pin.framesForwardKinematics(self.model, self.data, q)

    def GetJointState(self,joint_idx,return_type='homo'):

        rot=self.data.oMi[joint_idx].rotation
        trans=self.data.oMi[joint_idx].translation

        if return_type == "homo":
            
            homoMatrix=np.eye(4)
            homoMatrix[0:3,0:3]=rot
            homoMatrix[0:3,-1]=trans

            return homoMatrix

        elif return_type == "translation":

            return trans

        elif return_type == "rotation":

            return rot



if __name__ == "__main__":
    pass
