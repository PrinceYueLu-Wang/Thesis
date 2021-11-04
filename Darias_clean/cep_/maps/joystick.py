import torch
import numpy as np

from .base_map import Map
from cep_.utils import torch2numpy, numpy2torch
import time


class Joy_FK_ALL(Map):
    '''
    Map joints to cartesian space
    '''

    def __init__(self, kinematics_model):
        super(Joy_FK_ALL, self).__init__()
        self.kinematics = kinematics_model
        self.J = None

    def map_state(self, state):
        # x -> torch.size([1,14+2]) 7 q + 7 dq + 2 joystick aixs0 and axis1
        q = state[:, :7]  # torch.Size([1, 7])

        qd = state[:, 7:14]  # torch.Size([1, 7])

        qjoy = state[:, 14:]  # torch.Size([1, 2])

        q_np = torch2numpy(q[0, :])  # (7, )
        qd_np = torch2numpy(qd[0, :])  # (7, )

        ## Update Kinematics Model ##

        self.kinematics.update_kindyn(q_np)

        z = np.array(self.kinematics.links_fk(rotation=True))  # (7, 4, 4)
        J = np.array(self.kinematics.links_J())  # (7, 6, 7)
        zd = np.einsum('jnm,m->jn', J, qd_np)  # (7, 6) # TODO: ???

        self.J = numpy2torch(J)
        z = numpy2torch(z)  # torch.Size([7, 4, 4])
        zd = numpy2torch(zd)  # torch.Size([7, 6])
        zjoy = qjoy

        return [z, zd, zjoy]

    def map_action(self, action_origin):
        return torch.einsum('jnm,bm->bjn', self.J, action_origin)  # torch.Size([1000, 7, 6])
        #  J->torch.Size([7, 6, 7]), a->torch.Size([1000, 7])


class Joy_SelectionMap(Map):
    '''
    Map joints to cartesian space
    '''

    def __init__(self, idx=0):
        super(Joy_SelectionMap, self).__init__()
        self.idx = idx

    def map_state(self, x):
        # x[0]->torch.Size([7, 4, 4])
        # x[1]->torch.Size([7, 6])
        # x[2]->torch.Size([1,2])

        xyz = x[0][self.idx, ...]  # torch.Size([4, 4])
        v_xyz = x[1][self.idx, ...]  # torch.Size([6])

        joy_xyz = x[2]

        return [xyz, v_xyz, joy_xyz]

    def map_action(self, a):
        return a[:, self.idx, ...]


class Joy_SimpleMap(Map):
    '''
    Map joints to cartesian space
    '''

    def __init__(self):
        super(Joy_SimpleMap, self).__init__()

    def map_state(self, state):
        x = state[0]  # torch.Size([4, 4])
        dx = state[1]  # torch.Size([6])
        joy = state[2]  # torch.Size([2])

        return state

    def map_action(self, a):
        return a
