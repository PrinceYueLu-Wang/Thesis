import torch
import numpy as np

from .base_map import Map
from cep_.utils import torch2numpy, numpy2torch
import time


class JoyControlMap(Map):
    '''
    Map joints from to cartesian space
    '''
    def __init__(self, idx=0):
        super(SelectionMap, self).__init__()
        self.idx = idx

    def map_state(self, x):

        # x[0]->torch.Size([7, 4, 4])
        # x[1]->torch.Size([7, 6])

        xyz = x[0][self.idx, ...] # torch.Size([4, 4])
        v_xyz = x[1][self.idx, ...] # torch.Size([6])
        return [xyz, v_xyz]

    def map_action(self,a):
        return a[:, self.idx, ...]

class JoyFarMap(Map):
    '''
    Map joints from to cartesian space
    '''
    def __init__(self, idx=0):
        super(SelectionMap, self).__init__()
        self.idx = idx

    def map_state(self, x):

        # x[0]->torch.Size([7, 4, 4])
        # x[1]->torch.Size([7, 6])

        xyz = x[0][self.idx, ...] # torch.Size([4, 4])
        v_xyz = x[1][self.idx, ...] # torch.Size([6])
        return [xyz, v_xyz]

    def map_action(self,a):
        return a[:, self.idx, ...]

class JoyCloseMap(Map):
    '''
    Map joints from to cartesian space
    '''
    def __init__(self, idx=0):
        super(SelectionMap, self).__init__()
        self.idx = idx

    def map_state(self, x):

        # x[0]->torch.Size([7, 4, 4])
        # x[1]->torch.Size([7, 6])

        xyz = x[0][self.idx, ...] # torch.Size([4, 4])
        v_xyz = x[1][self.idx, ...] # torch.Size([6])
        return [xyz, v_xyz]

    def map_action(self,a):
        return a[:, self.idx, ...]

class forward_kin(Map):
    '''
    Map joints to cartesian space
    '''
    def __init__(self, kinematics_model):
        super(forward_kin, self).__init__()

        self.kinematics = kinematics_model
        self.J = None
        self.J

    def map_state(self, x):  # x: torch.Size([1, 14])
        q = x[:, :7]  # torch.Size([1, 7])
        qd = x[:, 7:]  # torch.Size([1, 7])
        q_np = torch2numpy(q[0, :])   # (7, )
        qd_np = torch2numpy(qd[0, :])  # (7, )

        ## Update Kinematics Model ##

        self.kinematics.update_kindyn(q_np)

        z = np.array(self.kinematics.links_fk(rotation=True))  # (7, 4, 4)
        J = np.array(self.kinematics.links_J())  # (7, 6, 7)
        zd = np.einsum('jnm,m->jn', J, qd_np)  # (7, 6) # TODO: ???

        self.J = numpy2torch(J)
        z = numpy2torch(z)  # torch.Size([7, 4, 4])
        zd = numpy2torch(zd)  # torch.Size([7, 6])
        return [z, zd]

    def map_action(self, a):
        return torch.einsum('jnm,bm->bjn', self.J, a)  # torch.Size([1000, 7, 6])
        #  J->torch.Size([7, 6, 7]), a->torch.Size([1000, 7])