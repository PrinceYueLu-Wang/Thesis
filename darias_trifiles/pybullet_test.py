import pybullet as p
import time
import pybullet_data

# 连接物理引擎
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
sphere_startPos = [0.5, 0, 1.]
sphere_startOrientation = p.getQuaternionFromEuler([0, 0, 0])

RobotId = p.loadURDF("/home/faye/Desktop/thesis/pinocchio/model/darias.urdf",startPos,startOrientation)
SphereId = p.loadURDF("/home/faye/Desktop/thesis/pinocchio/model/sphere_10cm.urdf",sphere_startPos,sphere_startOrientation)


time.sleep(10)
# for i in range(0,100):
#     # p.resetBasePositionAndOrientation(SphereId,[0.5,0,1+0.005*i],sphere_startOrientation)
#     # p.resetBasePositionAndOrientation(SphereId,[0.7,0.4-0.8/100*i,1.7-0.4/100*i],sphere_startOrientation)
#     p.resetJointState(RobotId,jointIndex=2,targetValue=0.01*i)
#     print(0.01*i)
#     p.stepSimulation()
#     time.sleep(0.2)


# for i in range(0,1):
#     wrp=p.getJointInfo(RobotId,i)
#     print("idx {} -- name {}".format(wrp[0],wrp[1]))
# # 按照位置和朝向重置机器人的位姿，由于我们之前已经初始化了机器人，所以此处加不加这句话没什么影响
# p.resetBasePositionAndOrientation(boxId, startPos, startOrientation)

# # 开始一千次迭代，也就是一千次交互，每次交互后停顿1/240
# for i in range(1000):
#     p.stepSimulation()
#     time.sleep(1 / 240)

# # 获取位置与方向四元数
# cubePos, cubeOrn = p.getBasePositionAndOrientation(boxId)
# print("-" * 20)
# print(f"机器人的位置坐标为:{cubePos}\n机器人的朝向四元数为:{cubeOrn}")
# print("-" * 20)

# # 断开连接
# p.disconnect()
