
import pybullet as p
import pybullet_data
from os.path import dirname, join, abspath
from time import sleep


class PyEnv():
    
    def __init__(self):

        self.CilentInit()

        self.JointInfoConfig()
        
        self.UrdfPathConfig()
        self.UrdfLoading()
        
    def JointInfoConfig(self):

        self.jointid_eef=7
        self.frameid_eef=19

        self.joint_num=7

        self.jointIdx_pinocchio=[1,2,3,4,5,6,7]
        self.frameIdx_pinocchio=[7,9,11,13,15,17,19]
        self.jointIdx_pybullet=[26,27,28,29,30,31,32]

    def CilentInit(self):

        physicsCilent = p.connect(p.GUI)
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        # p.setGravity(0, 0, -10)
        p.setGravity(0, 0, 0)

    def UrdfPathConfig(self):

        self.pkg_path=dirname(__file__)
        self.modelFolder=join(self.pkg_path,'../model')

        self.robotname="darias_singleArm.urdf"
        self.robotPath=join(self.modelFolder,self.robotname)

        self.spherePath=join(self.modelFolder,"sphere_10cm.urdf")
        self.sphereObsPath=join(self.modelFolder,"sphere_50cm.urdf")

    def UrdfLoading(self):

        # physicsCilent = p.connect(p.GUI)
        # # p.setAdditionalSearchPath(pybullet_data.getDataPath())
        # p.setGravity(0, 0, -10)
        planeId = p.loadURDF("plane.urdf")

        startPos = [0, 0, 0]
        startOrientation = p.getQuaternionFromEuler([0, 0, 0])
        self.robotID = p.loadURDF(self.robotPath,startPos,startOrientation)


        # startPos = [0, 0, 0]
        # startOrientation = p.getQuaternionFromEuler([0, 0, 0])
        # sphere_startPos = [0.5, -0.6, 1.8]
        # sphere_startOrientation = p.getQuaternionFromEuler([0, 0, 0])

        # SphereId = p.loadURDF(self.spherePath,sphere_startPos,sphere_startOrientation)

        sphere_obstaclePos = [0.62,  0.49  ,  1.52]
        sphere_obstacleOrientation = p.getQuaternionFromEuler([0, 0, 0])
        SphereObsId = p.loadURDF(
            self.sphereObsPath,
            basePosition=sphere_obstaclePos,
            baseOrientation=sphere_obstacleOrientation,
            useFixedBase=True
            )

    def EndEffectorAxis(self):
        pass


    def BulletUpdate(self,jointState_pinocchio):

        for (pino,bullet) in zip(self.jointIdx_pinocchio,self.jointIdx_pybullet):
                    p.resetJointState(self.robotID, jointIndex=bullet, targetValue=jointState_pinocchio[pino-1])
        

        # print(SphereObsId.getName())
        sleep(0.1)
        p.stepSimulation()
        

