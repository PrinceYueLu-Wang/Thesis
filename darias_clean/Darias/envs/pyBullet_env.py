
import pybullet as p
import pybullet_data
from os.path import dirname, join, abspath


class PyEnv():
    
    def __init__(self):
        pass

    def CilentInit(self):

        physicsCilent = p.connect(p.GUI)
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        p.setGravity(0, 0, -10)

    def UrdfLoading(self):

        self.pkg_path=dirname(__file__)
        self.robotFolder=join(self.pkg_path,'../model')
        self.robotname="darias_singleArm.urdf"
        self.robotPath=join(self.robotFolder,self.robotname)


    def LoadUrdf():

        physicsCilent = p.connect(p.GUI)
        # p.setAdditionalSearchPath(pybullet_data.getDataPath())
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
