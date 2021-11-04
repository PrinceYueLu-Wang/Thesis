import os,sys
dir=os.path.dirname(__file__)
rootlib=os.path.abspath(os.path.join(dir,'..'))
sys.path.append(rootlib)

import pybullet as p

physicsCilent = p.connect(p.GUI)
p.setGravity(0, 0, 0)

import time

# pkg_path=os.path.dirname(__file__)
# modelFolder=os.path.join(pkg_path,'../Daris/model')

# robotPath=os.path.join(modelFolder,'table.urdf')

startPos = [0, 0, 0]
startOrientation = p.getQuaternionFromEuler([0, 0, 0])

planeId = p.loadURDF('table.urdf',startPos,startOrientation)

time.sleep(100)

