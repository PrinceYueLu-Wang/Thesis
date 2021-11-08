import os
import pinocchio as pin
import numpy as np
import quaternion
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
        self.jointIdx_pybullet=[26,27,28,29,30,31,32]

        self.jointDict_pin2bullet=dict((pin,bullet) for pin,bullet in zip(self.jointIdx_pinocchio,self.jointIdx_pybullet))
        self.jointDict_bullet2pin=dict((bullet,pin) for bullet,pin in zip(self.jointIdx_pybullet,self.jointIdx_pinocchio))
        self.jointDict_pin2frame=dict((pin,frame) for pin,frame in zip(self.jointIdx_pinocchio,self.frameIdx_pinocchio))
    
    def JointBullet2Pin(self,jointIdx_Bullet):

        idx=self.jointIdx_pybullet.index(jointIdx_Bullet)
        idx=self.jointIdx_pinocchio[idx]
        return idx

    def JointPin2Bullet(self,jointIdx_pinocchio):
        iidx=self.jointIdx_pinocchio.index(jointIdx_pinocchio)
        idx=self.jointIdx_pybullet[idx]
        return idx

    def ModelInit(self,*args):
        if not args:
            # q=pin.randomConfiguration(self.model)
            q=pin.neutral(self.model)

            self.q_init=deepcopy(q)

            self.KinUpdate(q)
        else :
    
            self.q_init=deepcopy(args)

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

        J_inv=np.matmul(J.T,
             np.linalg.inv(
             np.matmul(J,J.T)
            +self.DAMP*np.eye(6)) )

        return J_inv

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

    def NeutralJointState(self):
        return pin.neutral(self.model)

    def JointSample(self):

        jointSpace=[]
        cordList=[]

        for j1 in np.linspace(0,np.pi,30):
            for j2 in np.linspace(0,np.pi,30):
                for j3 in np.linspace(0,np.pi,30):
                    for j4 in np.linspace(0,np.pi,30):
                        for j5 in np.linspace(0,np.pi,30):
                            for j6 in np.linspace(0,np.pi,30):
                                for j7 in np.linspace(0,np.pi,30):
                                    tmp=[j1,j2,j3,j4,j5,j6,j7]
                                    jointSpace.append(tmp)
        
        for q_item in jointSpace:
            self.KinUpdate(q_item)
            tmp_list=[]
            for i in range(0,7):
                trans=self.data.oMi[i].translation
                tmp_list.append(trans)

            cordList.append(tmp_list)

    def JointStateListUpdate(self):

        joint_state_list=[]

        for x in self.jointIdx_pinocchio:
            joint_state_list.append(
                self.GetJointState(x)
                )
        return joint_state_list

    def HomoFromRotTrans(self,args):

        quater=args[1]
        translation=np.array(args[0])

        quat=np.quaternion(quater[3],quater[0],quater[1],quater[2])
        rot_mat=quaternion.as_rotation_matrix(quat)

        T=np.eye(4)
        T[0:3,0:3]=rot_mat
        T[0:3,-1]=translation
        
        return T
if __name__ == "__main__":
    robot=Kinematic()
    # robot.JointSample()
