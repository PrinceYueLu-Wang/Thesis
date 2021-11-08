
import math
import random
from matplotlib import pyplot as plt
from matplotlib.patches import Circle
import time
import numpy as np
import quaternion

from Darias.controller import ControllSpeed_eef



class Vector3d():
    """
    3维向量, 支持加减, 支持常量乘法(右乘)
    """

    def __init__(self, x,y,z):
        self.deltaX = x
        self.deltaY = y
        self.deltaZ = z

        self.Vector3dUpdate()

    def Vector3dUpdate(self):

        self.vector_np= np.array([self.deltaX,
                                  self.deltaY,
                                  self.deltaZ])

        self.norm=np.linalg.norm(self.vector_np)

        if self.norm == 0 :
            self.unitvector_np=np.array([0.,0.,0.])
        else:
            self.unitvector_np = self.vector_np/self.norm

        self.unitvetor_x=self.unitvector_np[0]
        self.unitvetor_y=self.unitvector_np[1]
        self.unitvetor_z=self.unitvector_np[2] 

    def __add__(self, other):
        """
        + 重载
        :param other:
        :return:
        """
        vec = Vector3d(self.deltaX, self.deltaY,self.deltaZ)
        vec.deltaX += other.deltaX
        vec.deltaY += other.deltaY
        vec.deltaZ += other.deltaZ
        vec.Vector3dUpdate()
        return vec

    def __sub__(self, other):
        vec = Vector3d(self.deltaX, self.deltaY,self.deltaZ)
        vec.deltaX -= other.deltaX
        vec.deltaY -= other.deltaY
        vec.deltaZ -= other.deltaZ
        vec.Vector3dUpdate()
        return vec

    def __mul__(self, other):
        vec = Vector3d(self.deltaX, self.deltaY,self.deltaZ)
        vec.deltaX *= other
        vec.deltaY *= other
        vec.deltaZ *= other
        vec.Vector3dUpdate()
        return vec

    def __truediv__(self, other):
        return self.__mul__(1.0 / other)

    def __repr__(self):

        tmp = ''' Vector: \n deltaX:{} \n deltaY:{} \n deltaZ:{} \n norm : {} '''.format(self.deltaX, self.deltaY, self.deltaZ, self.norm)

        return tmp

class apf():

    def __init__(self,enableAttractive=False):

        self.enableAttractive=enableAttractive

        self.ParameterConfig()

        self.FieldConfig()

    def ParameterConfig(self):

        #default disable attractive force
        # self.enableAttractive=enableAttractive

        self.k_att = 1.0
        self.k_rep = 10.0

        self.step_size = 1
        self.max_iters = 2000
        
        self.iters = 0

        self.goal_threashold = 3*1e-3

        self.is_path_plan_success = False

        self.path=list()

        self.delta_t = 0.01


    def FieldConfig(self):

        # Target pose
        #======================================================#
        target_position=[0.75579,   -0.12606,   1.296]
        self.target_position = Vector3d(target_position[0],
                                   target_position[1],
                                   target_position[2])

        target_orientation=[0.21194,   0.7567,  -0.18032,  0.59158]

        quat=np.quaternion(target_orientation[3],target_orientation[0],
                           target_orientation[1],target_orientation[2])
            
        rot_mat=quaternion.as_rotation_matrix(quat)

        T_target_world=np.eye(4)
        T_target_world[0:3,0:3]=rot_mat
        T_target_world[0:3,-1]=target_position

        self.T_target_world=T_target_world

        # Obstacle
        #======================================================#
        obstacle=[0.62,   0.49,   1.52]  
        self.obstacle=Vector3d(obstacle[0],obstacle[1],obstacle[2])
        self.obsRadius=0.5

        self.rr=self.obsRadius+0.05

        # Init Pose
        #======================================================#
        # initpos=[0.479,    1.0802,    1.5]
        # self.currentpos=Vector3d(initpos[0],initpos[1],initpos[2])


        #======================================================#

        self.forceRepArrow=[]

    
    def Attractive(self):
        

        # forceAtt=self.target_position-self.currentpos
        # forceAtt shape = (6,)
        forceAtt=ControllSpeed_eef(self.T_target_world,self.T_body)

        return forceAtt

    def Repulsive(self):

        vecToObs=self.currentpos - self.obstacle

        distToObs=vecToObs.norm

        if distToObs > self.obsRadius +self.rr:
            
            forceRep=Vector3d(0. ,0. ,0. )

        else:

            forceRep=Vector3d(vecToObs.unitvetor_x,
                              vecToObs.unitvetor_y,
                               vecToObs.unitvetor_z) 
                               
            forceRep=forceRep* self.k_rep * (1.0 / vecToObs.norm - 1.0 / self.rr)

            forceRep=forceRep / (vecToObs.norm ** 2)

        return forceRep

    def ForceUpdate(self,currentpose: np.ndarray):

        # currentpos 4 by 4 homo matrix
        # return force 6 by 1

        translation=currentpose[0:3,-1]

        self.T_body=currentpose

        self.currentpos=Vector3d(translation[0],translation[1],translation[2])

        if self.enableAttractive:

            #here forceRep shape=(6,)
            forceRep=np.zeros(shape=(6,))
            forceRep[0:3]=self.Repulsive().vector_np
      
            forceSum=self.Attractive()+forceRep

        else:

            forceRep=np.zeros(shape=(6,))
            forceRep[0:3]=self.Repulsive().vector_np

            forceSum=forceRep
        
        return 0.1*forceSum



