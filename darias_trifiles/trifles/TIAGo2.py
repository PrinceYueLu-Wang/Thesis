# import pinocchio as pin
# import numpy as np
# import pybullet as p
# import pybullet_data
# import torch
# from torch import tensor
# from pinocchio.robot_wrapper import RobotWrapper
# import os
# import matplotlib.pyplot as plt
# import time
# from cep.utils import numpy2torch, torch2numpy
# from cep.liegroups.torch import SO3, SE3


# global lego_x
# global lego_y
# global lego_z

# global x_text
# global y_text
# global z_text

# global l1  # End-effector x in WORLD frame visualization
# global l2  # End-effector y in WORLD frame visualization
# global l3  # End-effector z in WORLD frame visualization

# global w1  # End-effector x frame visualization
# global w2  # End-effector y frame visualization
# global w3  # End-effector z frame visualization


# class Tiago_OSC:

#     def __init__(self, start_point=None):

#         # Load tiago from pinocchio
#         self.robot = self.load_tiago()
#         print('robot.model: ', self.robot)

#         self.number_iteration = 500  # Define max iteration number
#         self.dt = 0.01  # Define time step

#         self.first_time = True

#         # TODO: Set the desired points
#         self.qua = pin.Quaternion(0., 0., 0., 1.)
#         self.qua.normalize()
#         # self.start_point = np.array([0.1, -0.7, 0.7])  # 0.10805 -0.7345  0.7065
#         self.start_point = start_point
#         self.end_point = np.array([0.8, 0., 0.8])

#         # Position X difference btw start and end point
#         difference_x = self.end_point[0] - self.start_point[0]
#         # Position Y difference btw start and end point
#         difference_y = self.end_point[1] - self.start_point[1]
#         # Position Z difference btw start and end point
#         difference_z = self.end_point[2] - self.start_point[2]

#         # TODO: Generate a simple trajectory
#         self.traj_points_num = 10  # Define how many points along the trajectory
#         self.traj_points = []  # positions (3x1 array) of trajectory points
#         self.traj_des = []  # position and oriention (SE3) of trajectory points

#         for k in range(self.traj_points_num):
#             self.traj_points.append(self.start_point.copy())
#             tmp = pin.SE3(self.qua, self.start_point)
#             tmp.rotation = np.eye(3)
#             self.traj_des.append(tmp)

#             self.start_point[0] = self.start_point[0] + \
#                 (difference_x / (self.traj_points_num - 1))
#             self.start_point[1] = self.start_point[1] + \
#                 (difference_y / (self.traj_points_num - 1))
#             self.start_point[2] = self.start_point[2] + \
#                 (difference_z / (self.traj_points_num - 1))
#         self.start_point = np.array([0.1, -0.7, 0.7])

#         # TODO: set desired position and orientation
#         self.x_des = pin.SE3(self.qua, self.start_point)

#         # Get 7th arm joint index and end-effector index
#         self.JOINT_INX = 7
#         self.EE_idx = self.robot.model.getFrameId('arm_7_link')
#         self.ERROR = 1e-4
#         self.DISRANCE = 0.02

#     def MoveIt_generate_traj(self):  # TODO: 06.14

#         return

#     # Function to get the position/velocity of all joints from pybullet
#     def getPosVelJoints(self, robotId, joint_indexes):

#         # State of all joints (position, velocity, reaction forces, appliedJointMotortoruqe)
#         jointStates = p.getJointStates(robotId, joint_indexes)
#         # print('np.shape(jointStates):', np.shape(jointStates))
#         # print('len(jointStates): ', len(jointStates))
#         # print('jointStates[0]: ', jointStates[0])
#         # print('jointStates[0][0]: ', jointStates[0][0])
#         # baseState = p.getBasePositionAndOrientation(robotId)  # Position and orientation of the free flying base (Position, orientation)
#         # baseVel = p.getBaseVelocity(robotId)  # Velocity of the free flying base  (linear velocity, angular velocity)

#         # Reshaping data into q and qdot
#         joint_pos = np.vstack((np.array(
#             [[jointStates[i_joint][0] for i_joint in range(len(jointStates))]]).transpose()))
#         # q = np.vstack((np.array([baseState[0]]).transpose(), np.array([baseState[1]]).transpose(),
#         #                np.array([[jointStates[i_joint][0] for i_joint in range(len(jointStates))]]).transpose()))
#         # print('q: ', q) # ([:3] -> base position,
#         #                 # [3:7] -> base orientation in Quatenion,
#         #                 # [7:9] -> wheel right and left,
#         #                 # [9:16] -> position of 7 joints,
#         #                 # [16:] -> position of gripper right finger and left finger
#         # print('np.shape(q): ', np.shape(q)) # (18, 1), baseState[1] is orientation in Quatenion

#         return joint_pos

#     def add_debug_lines(self, X_w, Y_w, Z_w, XYZ_ee):

#         global l1
#         global l2
#         global l3

#         global w1
#         global w2
#         global w3

#         if self.first_time:
#             l1 = p.addUserDebugLine([0, 0, 0], X_w, [1, 0, 0], lineWidth=2)
#             l2 = p.addUserDebugLine([0, 0, 0], Y_w, [0, 1, 0], lineWidth=2)
#             l3 = p.addUserDebugLine([0, 0, 0], Z_w, [0, 0, 1], lineWidth=2)

#             w1 = p.addUserDebugLine(XYZ_ee, X_w, [1, 0, 0], lineWidth=2)
#             w2 = p.addUserDebugLine(XYZ_ee, Y_w, [0, 1, 0], lineWidth=2)
#             w3 = p.addUserDebugLine(XYZ_ee, Z_w, [0, 0, 1], lineWidth=2)
#         else:
#             p.removeUserDebugItem(l1)
#             p.removeUserDebugItem(l2)
#             p.removeUserDebugItem(l3)

#             p.removeUserDebugItem(w1)
#             p.removeUserDebugItem(w2)
#             p.removeUserDebugItem(w3)

#             l1 = p.addUserDebugLine([0, 0, 0], X_w, [1, 0, 0], lineWidth=2)
#             l2 = p.addUserDebugLine([0, 0, 0], Y_w, [0, 1, 0], lineWidth=2)
#             l3 = p.addUserDebugLine([0, 0, 0], Z_w, [0, 0, 1], lineWidth=2)

#             w1 = p.addUserDebugLine(XYZ_ee, X_w, [1, 0, 0], lineWidth=2)
#             w2 = p.addUserDebugLine(XYZ_ee, Y_w, [0, 1, 0], lineWidth=2)
#             w3 = p.addUserDebugLine(XYZ_ee, Z_w, [0, 0, 1], lineWidth=2)

#         return l1, l2, l3, w1, w2, w3

#     def add_xyz_text(self, X_w, Y_w, Z_w):

#         global x_text
#         global y_text
#         global z_text

#         if self.first_time:
#             x_text = p.addUserDebugText('X', X_w)
#             y_text = p.addUserDebugText('Y', Y_w)
#             z_text = p.addUserDebugText('Z', Z_w)

#         return x_text, y_text, z_text

#     def put_obj_in_world(self, X_w, Y_w, Z_w):

#         global lego_x
#         global lego_y
#         global lego_z

#         if self.first_time:
#             lego_x = p.loadURDF('sphere_small.urdf', X_w, p.getQuaternionFromEuler([0, 0, 0]),
#                                 useFixedBase=True, useMaximalCoordinates=True)
#             lego_y = p.loadURDF('cube_small.urdf', Y_w, p.getQuaternionFromEuler([0, 0, 0]),
#                                 useFixedBase=True, useMaximalCoordinates=True)
#             lego_z = p.loadURDF('/lego/lego.urdf', Z_w, p.getQuaternionFromEuler([0, 0, 0]),
#                                 useFixedBase=True, useMaximalCoordinates=True)

#         return lego_x, lego_y, lego_z

#     def calculate_xyz_world(self, T: tensor):

#         vec_x_e = tensor([[.33, 0., 0., 1.]]).reshape(4, 1)
#         vec_y_e = tensor([[0., .33, 0., 1.]]).reshape(4, 1)
#         vec_z_e = tensor([[0., 0., .33, 1.]]).reshape(4, 1)

#         vec_x_w = torch2numpy(torch.matmul(T, vec_x_e)).tolist()
#         vec_y_w = torch2numpy(torch.matmul(T, vec_y_e)).tolist()
#         vec_z_w = torch2numpy(torch.matmul(T, vec_z_e)).tolist()

#         X_w, Y_w, Z_w = [], [], []
#         for i in range(3):
#             X_w.append(vec_x_w[i][0])
#             Y_w.append(vec_y_w[i][0])
#             Z_w.append(vec_z_w[i][0])

#         return X_w, Y_w, Z_w

#     def joint_control(self, robot, q, K: int, dt: float):

#         q_des = np.ones((robot.nq, 1)) * 0.6
#         dq = K * (q - q_des)
#         ans_q = q + dq * dt

#         return ans_q

#     def start_pybullet(self):  # load Tiago in Pybullet

#         # or p.DIRECT for non-graphical version
#         physicsClient = p.connect(p.GUI)
#         p.setAdditionalSearchPath(pybullet_data.getDataPath())
#         p.resetSimulation(p.RESET_USE_DEFORMABLE_WORLD)
#         p.setGravity(0, 0, -9.81)

#         base_dir = os.path.abspath(os.path.dirname(__file__) + '../../..')
#         robot_dir = os.path.join(base_dir, 'robots/tiago/')
#         urdf_filename = os.path.join(robot_dir, 'tiago_single_modified.urdf')

#         planeId = p.loadURDF("plane.urdf")
#         startPos = [0., 0., 0.]
#         startOrientation = [0., 0., 0.]
#         robotId = p.loadURDF(urdf_filename, startPos, p.getQuaternionFromEuler(
#             [0., 0., 0.]), useFixedBase=1)
#         # p.loadURDF("cube_small.urdf", np.array([0.4, -0.5, 0.5]),
#         #            p.getQuaternionFromEuler([0, 0, 0]),
#         #            useFixedBase=True, useMaximalCoordinates=True)
#         p.loadURDF('sphere_1cm.urdf', np.array([0.8, 0., 0.8]),  # TODO: Put an object in target postion
#                    p.getQuaternionFromEuler([0, 0, 0]),
#                    useFixedBase=True)

#         joint_indexes = [31, 32, 33, 34, 35, 36, 37]

#         return robotId, planeId, joint_indexes

#     def plot_mu(self, mu_values: list, num: int):

#         fig, axs = plt.subplots(2, 3)
#         t = np.arange(0, num, 1)

#         wx = []
#         wy = []
#         wz = []
#         vx = []
#         vy = []
#         vz = []

#         for i in range(num):
#             wx.append(mu_values[i][0].item())
#             wy.append(mu_values[i][1].item())
#             wz.append(mu_values[i][2].item())
#             vx.append(mu_values[i][3].item())
#             vy.append(mu_values[i][4].item())
#             vz.append(mu_values[i][5].item())

#         axs[0, 0].plot(t, wx)
#         axs[0, 0].set_title('wx: task space')
#         axs[0, 0].set_ylim(min(wx)-10, max(wx)+10)

#         axs[0, 1].plot(t, wx)
#         axs[0, 1].set_title('wy: task space')
#         axs[0, 1].set_ylim(min(wy)-10, max(wy)+10)

#         axs[0, 2].plot(t, wx)
#         axs[0, 2].set_title('wz: task space')
#         axs[0, 2].set_ylim(min(wz)-10, max(wz)+10)

#         axs[1, 0].plot(t, wx)
#         axs[1, 0].set_title('vx: task space')
#         axs[1, 0].set_ylim(min(vx)-10, max(vx)+10)

#         axs[1, 1].plot(t, wx)
#         axs[1, 1].set_title('vy: task space ')
#         axs[1, 1].set_ylim(min(vy)-10, max(vy)+10)

#         axs[1, 2].plot(t, wx)
#         axs[1, 2].set_title('vz: task space')
#         axs[1, 2].set_ylim(min(vz)-10, max(vz)+10)

#         plt.show()

#     def plot_joints(self, joint_values: list, joint_pos_values: list, num: int):

#         fig, axs = plt.subplots(2, 4)
#         t = np.arange(0, num, 1)

#         j1 = []
#         j2 = []
#         j3 = []
#         j4 = []
#         j5 = []
#         j6 = []
#         j7 = []

#         # j1_pos = []
#         # j2_pos = []
#         # j3_pos = []
#         # j4_pos = []
#         # j5_pos = []
#         # j6_pos = []
#         # j7_pos = []

#         for i in range(self.number_iteration):
#             j1.append(joint_values[i][0])
#             j2.append(joint_values[i][1])
#             j3.append(joint_values[i][2])
#             j4.append(joint_values[i][3])
#             j5.append(joint_values[i][4])
#             j6.append(joint_values[i][5])
#             j7.append(joint_values[i][6])

#             # j1_pos.append(joint_pos_values[i][0])
#             # j2_pos.append(joint_pos_values[i][1])
#             # j3_pos.append(joint_pos_values[i][2])
#             # j4_pos.append(joint_pos_values[i][3])
#             # j5_pos.append(joint_pos_values[i][4])
#             # j6_pos.append(joint_pos_values[i][5])
#             # j7_pos.append(joint_pos_values[i][6])

#         #q1_des = np.ones((number_iteration, 1)) * np.pi/3
#         axs[0, 0].plot(t, j1)
#         #axs[0, 0].plot(t, j1_pos, '+', linewidth=.02)
#         axs[0, 0].set_title('1st Joint')
#         axs[0, 0].set_ylim(min(j1) - 1, max(j1) + 1)
#         #axs[0, 0].plot(t, q1_des, label='Desired q=pi/3')

#         axs[0, 1].plot(t, j2)
#         #axs[0, 1].plot(t, j2_pos, '+', linewidth=.02)
#         axs[0, 1].set_title('2nd Joint')
#         axs[0, 1].set_ylim(min(j2) - 1, max(j2) + 1)

#         axs[0, 2].plot(t, j3)
#         #axs[0, 2].plot(t, j3_pos, '+', linewidth=.02)
#         axs[0, 2].set_title('3rd Joint')
#         axs[0, 2].set_ylim(min(j3) - 1, max(j3) + 1)

#         axs[0, 3].plot(t, j4)
#         #axs[0, 3].plot(t, j4_pos, '+', linewidth=.02)
#         axs[0, 3].set_title('4th Joint')
#         axs[0, 3].set_ylim(min(j4) - 1, max(j4) + 1)

#         axs[1, 0].plot(t, j5)
#         #axs[1, 0].plot(t, j5_pos, '+', linewidth=.02)
#         axs[1, 0].set_title('5th Joint')
#         axs[1, 0].set_ylim(min(j5) - 1, max(j5) + 1)

#         axs[1, 1].plot(t, j6)
#         #axs[1, 1].plot(t, j6_pos, '+', linewidth=.02)
#         axs[1, 1].set_title('6th Joint')
#         axs[1, 1].set_ylim(min(j6) - 1, max(j6) + 1)

#         axs[1, 2].plot(t, j7)
#         #axs[1, 2].plot(t, j7_pos, '+', linewidth=.02)
#         axs[1, 2].set_title('7th Joint')
#         axs[1, 2].set_ylim(min(j7) - 1, max(j7) + 1)

#         plt.show()

#     def plot_euclidiean_dist(self, dist_values, num):

#         fig, axs = plt.subplots(1, 1)
#         t = np.arange(0, num, 1)
#         axs.plot(t, dist_values)
#         axs.set_title('Euclidean distance')
#         plt.show()

#     def plot_error(self, error_values, num):

#         fig, axs = plt.subplots(1, 1)
#         t = np.arange(0, num, 1)
#         axs.plot(t, error_values)
#         axs.set_title('Error')
#         plt.savefig('Error')
#         plt.show()

#     def plot_xyz(self, x_values: list, y_values: list, z_values: list, num: int):

#         fig, axs = plt.subplots(1, 3)
#         t = np.arange(0, num, 1)

#         x_des = np.ones((num, )) * 0.8
#         axs[0].plot(t, x_values, label='Current x')
#         axs[0].set_title('X')
#         axs[0].plot(t, x_des, label='Desired x')
#         axs[0].legend()
#         axs[0].set_ylim(min(x_values) - 0.1, max(x_values) + 0.1)

#         y_des = np.ones((num, )) * 0.
#         axs[1].plot(t, y_values, label='Current y')
#         axs[1].set_title('Y')
#         axs[1].plot(t, y_des, label='Desired y')
#         axs[1].set_ylim(min(y_values) - .3, max(y_values) + .3)

#         z_des = np.ones((num, )) * 0.8
#         axs[2].plot(t, z_values, label='Current z')
#         axs[2].set_title('Z')
#         axs[2].plot(t, z_des, label='Desired z')
#         axs[2].set_ylim(min(z_values) - .3, max(z_values) + .3)

#         plt.show()

#     def se3ToTransfrom(self, SE3):

#         # Transform a SE3 to a  (4, 4) transformation matrix.

#         r = numpy2torch(SE3.rotation)
#         t = numpy2torch(SE3.translation)
#         x1 = torch.cat((r, t.reshape(3, 1)), 1)
#         homo = torch.tensor([[0, 0, 0, 1]])
#         Tf = torch.cat((x1, homo), 0)

#         return Tf

#     def load_tiago(self):  # TODO: Modify the urdf path

#         base_dir = os.path.abspath(os.path.dirname(__file__) + '../../..')
#         robot_dir = os.path.join(base_dir, 'robots/tiago/')
#         urdf_filename = os.path.join(robot_dir, 'tiago_single_modified.urdf')
#         robot = RobotWrapper.BuildFromURDF(urdf_filename, [robot_dir])
#         # robot.initViewer()
#         return robot

#     def set_context(self, state, R):

#         x = state[0]  # Tensor(4, 4), end-effector rotation and position SE(3)
#         v = state[1]  # Tensor (1, 6), end-effector spatial velocity V_b
#         print('state:', state)
#         # print('x: ', x)
#         # print('v: ', v)

#         # print('self.R_inv: ', self.R_inv) # Tensor (4, 4)
#         # print('R: ', self.R) # Tensor (4, 4)
#         R_inv = torch.inverse(R)
#         Htl = torch.matmul(R_inv, x)  # R_inv * X
#         print('Htl: ', Htl)
#         # <cep.liegroups.torch.se3.SE3Matrix>, SE(3)
#         Xe = SE3.from_matrix(Htl, normalize=True)
#         print('Xe: ', Xe)
#         xtl = Xe.log()  # Tensor(1, 6), (omega, V)
#         print('xtl: ', xtl)
#         vtl = -xtl
#         A = SE3.from_matrix(R)  # <cep.liegroups.torch.se3.SE3Matrix>, SE(3), R
#         print('A: ', A)
#         # Adjoint map (Spatial velocity from one frame to another frame), Tensor (6,6),
#         Adj_lw = A.adjoint()
#         print('Adj_lw: ', Adj_lw)
#         ve_w = torch.matmul(Adj_lw, vtl)  # Tensor(6, 1)
#         print('v_ew: ', ve_w)
#         ###########################################

#         scale = 20.
#         mu = scale * ve_w - 1.2 * scale * v

#         return mu
#         # print('mu: ', mu)  # Tensor(6, 1)

#     def calculate_mu(self, state, R):  # TODO: Acceleration control
#         '''
#         params:
#             state: Tensor -> contains end-effector rotation and position s[0], spatial velocity s[1]
#             R: Tensor (4, 4) -> Homogenous transformation matrix of end-effector
#         return:
#             mu: Tensor (1, 6). ddx, contains (dw, dv),
#                 then ddq = J_pinv * ddx, and dq = dq + ddq * dt
#                                              q = q + dq * dt
#         '''

#         x = state[0]  # Tensor(4, 4), end-effector rotation and position SE(3)
#         v = state[1]  # Tensor (1, 6), end-effector spatial velocity V_b
#         # index = [3, 4, 5, 0, 1, 2]
#         # v = v[index]

#         R_inv = torch.inverse(R)
#         Htl = torch.matmul(R_inv, x)  # R_inv * X
#         # <cep.liegroups.torch.se3.SE3Matrix>, SE(3)
#         Xe = SE3.from_matrix(Htl, normalize=True)
#         xtl = Xe.log()  # Tensor(1, 6), (omega, V)
#         vtl = -xtl

#         A = SE3.from_matrix(R)
#         Adj_lw = A.adjoint()
#         ve_w = torch.matmul(Adj_lw, vtl)

#         # TODO: Acceleration control
#         scale = 20
#         mu = scale * ve_w - 1.2 * scale * v

#         return mu

#     def calculate_vtl(self, state, R):  # TODO: Velocity control
#         '''
#         params:
#             state: Tensor -> contains end-effector rotation and position s[0], spatial velocity s[1]
#             R: Tensor (4, 4) -> Homogenous transformation matrix of end-effector
#         return:
#             vtl: Tensor (1, 6). dx, contains (w, v), then dq = J_pinv * dx, and q = q + dq * dt
#         '''

#         x = state[0]  # Tensor(4, 4), end-effector rotation and position SE(3)
#         # v = state[1]  # Tensor (1, 6), end-effector spatial velocity V_b

#         R_inv = torch.inverse(R)
#         Htl = torch.matmul(R_inv, x)  # R_inv * X
#         # <cep.liegroups.torch.se3.SE3Matrix>, SE(3)
#         Xe = SE3.from_matrix(Htl, normalize=True)
#         xtl = Xe.log()  # Tensor(1, 6), (omega, V)
#         vtl = -xtl

#         A = SE3.from_matrix(R)
#         Adj_lw = A.adjoint()
#         ve_w = torch.matmul(Adj_lw, vtl)

#         return ve_w

#     def start_simulation(self):

#         # Initialize q, dq, desired pose R and error threshold
#         q = pin.neutral(self.robot.model)
#         dq = np.zeros(self.robot.nv)
#         # R = se3ToTransfrom(x_des)

#         # load Tiago from pybullet
#         self.robotId, self.planeId, self.joint_indexes = self.start_pybullet()

#         self.q_des = np.ones((self.robot.nq,)) * 0.  # joint control desired q

#         # for jj in range(len(joint_indexes)):  # TODO: PYBULLET set joint positions
#         #     p.resetJointState(robotId, joint_indexes[jj], q_des[jj])

#         p.addUserDebugLine(self.start_point.tolist(),
#                            self.end_point.tolist(),
#                            [0.5, 0.4, 1],
#                            lineWidth=3)
#         p.addUserDebugText('Start point', self.start_point.tolist(), [0, 1, 1])
#         p.addUserDebugText('End point', self.end_point.tolist(), [0, 1, 1])

#         # for x_des in traj_des:  # TODO: For all the points along trajectory
#         #
#         #     print('==================')
#         R = self.se3ToTransfrom(self.x_des)

#         # TODO: Store data of joints in pinocchio, positions, error, joints in pybullet, mu(ddx in task space)
#         joint_values = []
#         x_values = []
#         y_values = []
#         z_values = []
#         error_values = []
#         joint_pos_values = []
#         mu_values = []
#         dist_values = []

#         # TODO: Start update loop
#         for ii in range(self.number_iteration):

#             print("============= Start Update ", self.x_des,
#                   '=============', ii, "th =============")

#             # TODO: Check the initial 7 q values in Pinocchio
#             print('1 q: ', q)

#             # TODO: Check the initial  7 joint states in PYBULLET
#             check_joint_states = self.getPosVelJoints(
#                 self.robotId, self.joint_indexes)
#             # print('check_joint_states: ', check_joint_states)

#             # TODO: Update jacobian and Forward kinematics
#             pin.computeJointJacobians(self.robot.model, self.robot.data, q)
#             pin.forwardKinematics(self.robot.model, self.robot.data, q)
#             pin.framesForwardKinematics(self.robot.model, self.robot.data, q)
#             # pin.updateFramePlacements(robot.model, robot.data)

#             # TODO: Get current position and orientation
#             # current position and orientation of EE
#             cur_x = self.robot.data.oMf[self.EE_idx]
#             print('***cur_x: ', cur_x)
#             # EE_pos_link = p.getLinkState(robotId, 37)
#             joint_states = self.getPosVelJoints(
#                 self.robotId, self.joint_indexes)
#             # print('***EE_pos_link: ', EE_pos_link)
#             # print('***joint_states: ', joint_states)
#             # print('2 q: ', q)

#             # TODO: Get current state(Position and orientation, velocity)
#             cur_dx_vec = pin.getFrameVelocity(
#                 self.robot.model, self.robot.data, self.EE_idx, pin.ReferenceFrame.WORLD).vector
#             # To 4x4 Transformation matrix #
#             cur_x = self.se3ToTransfrom(cur_x)
#             cur_dx = numpy2torch(cur_dx_vec)
#             state = [cur_x, cur_dx]

#             # TODO: Velocity control
#             v_ew = self.calculate_vtl(state=state, R=R)

#             # TODO: Acceleration control
#             mu = self.calculate_mu(state=state, R=R)
#             #print('mu: ', mu)

#             # TODO: RECORD ddx in task space
#             mu_values.append(mu)

#             # TODO: Compute J of end-effector of WORLD frame
#             # pin.framesForwardKinematics(robot.model, robot.data, q)
#             # pin.forwardKinematics(robot.model, robot.data, q)
#             pin.computeJointJacobians(self.robot.model, self.robot.data, q)
#             pin.updateFramePlacements(self.robot.model, self.robot.data)
#             J = pin.getFrameJacobian(
#                 self.robot.model, self.robot.data, self.EE_idx, pin.ReferenceFrame.WORLD)
#             J = numpy2torch(J)
#             #print('Jacobian: ', J)

#             # TODO: Velocity control
#             damp = 1e-6  # 0.1, 0.01, 0.001
#             Idt = numpy2torch(np.eye(6))
#             JJ_vel = torch.matmul(J.T, torch.inverse(torch.matmul(
#                 J, J.T) + (damp**2 * Idt)))  # TODO: Add damped pseudoinverse
#             dq = torch.matmul(JJ_vel, v_ew)
#             # dq = torch.matmul(torch.pinverse(J), v_ew)
#             dq = torch2numpy(dq)
#             # Euler discretization
#             # dt = .01
#             # q = q + dq * dt
#             # q = pin.integrate(robot.model, q, dq * dt)

#             # TODO: Acceleration control
#             JJ_acc = torch.matmul(J.T, torch.inverse(torch.matmul(
#                 J, J.T) + (damp ** 2 * Idt)))  # TODO: Add damped pseudoinverse
#             ddq = torch.matmul(JJ_acc, mu)
#             ddq = torch2numpy(ddq)
#             dt = 0.01
#             dq = dq + ddq * dt
#             q = q + dq * dt
#             # q = pin.integrate(robot.model, q, dq * dt)

#             # TODO: CHECK->print desired pose and current pose
#             # print('3 q: ', q)
#             # print('dq: ', dq)
#             # print('###desired pose: ', x_des)
#             # print('###current pose: \n', cur_x)

#             # Set constraint between plane and robot base
#             # p.createConstraint(planeId, -1, robotId, -1,
#             #                    p.JOINT_FIXED,
#             #                    [0, 0, 0], [0, 0, 0], [0, 0, 0])

#             #pin.computeAllTerms(robot.model, robot.data, q, dq)
#             # p.setJointMotorControlArray(robotId, joint_indexes, p.POSITION_CONTROL, q)

#             # Update jacobian and Forward kinematics
#             # pin.computeJointJacobians(robot.model, robot.data, q)
#             # pin.framesForwardKinematics(robot.model, robot.data, q)
#             # # pin.forwardKinematics(robot.model, robot.data, q)
#             # pin.updateFramePlacements(robot.model, robot.data)

#             # How to control
#             # p.setJointMotorControlArray(robotId, joint_indexes, p.POSITION_CONTROL, q)
#             # p.setJointMotorControlArray(robotId, joint_indexes, p.VELOCITY_CONTROL, dq)

#             # TODO: PYBULLET set joint positions
#             for jj in range(len(self.joint_indexes)):
#                 p.resetJointState(self.robotId, self.joint_indexes[jj], q[jj])

#             # TODO: RECORD joint positions from Pinocchio, XYZ position, joint values from PYBULLET
#             joint_values.append(q)

#             x_values.append(self.robot.data.oMf[self.EE_idx].translation[0])
#             y_values.append(self.robot.data.oMf[self.EE_idx].translation[1])
#             z_values.append(self.robot.data.oMf[self.EE_idx].translation[2])

#             joint_pos = self.getPosVelJoints(self.robotId, self.joint_indexes)
#             joint_pos_values.append(joint_pos)

#             # TODO: Visualize homogeneous transformation
#             X_w, Y_w, Z_w = self.calculate_xyz_world(
#                 self.se3ToTransfrom(self.robot.data.oMf[self.EE_idx]))
#             # lego_x, lego_y, lego_z = put_obj_in_world(X_w, Y_w, Z_w)  # TODO: put 3 objects on world frame
#             # x_text, y_text, z_text = add_xyz_text(X_w, Y_w, Z_w)
#             H = torch.matmul(cur_x, torch.tensor(
#                 [[0., 0., 0., 1.]]).reshape(4, 1))
#             l1, l2, l3, w1, w2, w3 = self.add_debug_lines(X_w, Y_w, Z_w, H[:3])
#             # H[:3], roboot.data.oMi[JOINT_IDX], robot.data.oMf[EE_idx]
#             first_time = False

#             # TODO: PYBULLET STEPSIMULATION
#             p.stepSimulation()
#             # time.sleep(1./100.)

#             # TODO: Check error (Sum of position difference between current and target), and euclidean distance
#             # Transformation from current EE to desired EE
#             dMi = self.x_des.actInv(self.robot.data.oMf[self.EE_idx])
#             #err = np.linalg.norm(pin.log6(dMi).vector)
#             err = abs(
#                 np.sum(self.robot.data.oMf[self.EE_idx].translation - self.x_des.translation))
#             distance = np.sqrt(np.sum(np.power(np.abs(
#                 self.robot.data.oMf[self.EE_idx].translation - self.x_des.translation), 2)))
#             print('position error: ', err)
#             print('Distance: ', distance)

#             # TODO: RECORD error
#             error_values.append(err)
#             dist_values.append(distance)

#             # TODO: Check convergence
#             if distance < self.DISRANCE:
#                 print('q result: ', q)
#                 print('dq result: ', dq)
#                 print('Final pose: ', self.robot.data.oMi[self.JOINT_INX])
#                 print("Convergence achieved!")
#                 number_iteration = ii+1
#                 break

#             # # # TODO: Plot
#             # plot_mu(mu_values, number_iteration)
#             # plot_joints(joint_values, joint_pos_values, number_iteration)
#             # plot_error(error_values, number_iteration)
#             # plot_euclidiean_dist(dist_values, number_iteration)
#             # plot_xyz(x_values, y_values, z_values, number_iteration)

# #
# # if __name__ == "__main__":
# #     T = Tiago_OSC()
# #     T.start_simulation(start_point)
