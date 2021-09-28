import torch
import torch.distributions as tdist
import numpy as np

from .energy_leaf import EnergyLeaf

from cep_.utils import eul2rot, rot2eul, rot2quat
from cep_.liegroups.torch import SO3, SE3


class TaskGoTO_JoyControl(EnergyLeaf):
    '''
        state : 3 part for endeffector
        state[0] : x
        state[1] : dx
        state[2] : joystick axis0 and axis1
    '''

    def __init__(self, dim=6, A=None, b=None, R=None, var=None):
        super(TaskGoTO_JoyControl, self).__init__()
        self.dim = dim

        if A is None:
            A = torch.eye(self.dim).float()
        self.register_buffer('A', A)
        if b is None:
            b = torch.zeros(3).float()
        self.register_buffer('b', b)
        if var is None:
            var = torch.eye(self.dim).float()
        self.register_buffer('var', var)

        if R is None:
            R = torch.eye(3).float()

        R_inv = torch.inverse(R)
        self.register_buffer('R', R)
        self.register_buffer('R_inv', R_inv)

        self.p_dx = None

    def set_context(self, state):
        '''
        We compute the conditioning variables of our model to have a faster optimization
        '''
        x = state[0]  # Tensor(4, 4), end-effector rotation and position SE(3)
        v = state[1]  # Tensor (1, 6), end-effector spatial velocity V_b
        joystick = state[2]
        # ===========================================#

        # x0 = state[0][0, -1]
        # y0 = state[0][1, -1]

        # k_x=(x0 - 0.3)
        # k_y=(y0 - 0.)

        # dx = -k_x
        # dy = -k_y

        dy = joystick[0][0] * 2.0
        dx = joystick[0][1] * 1.0

        mu = torch.zeros((6))  # Tensor(6, 1)
        mu[0] = dx
        mu[1] = dy

        _var = torch.clone(self.var)

        for i in range(2, 6):
            _var[i, i] = _var[i, i] * 0.1

        self.p_dx = tdist.MultivariateNormal(mu, _var)  # self.var->torch.size(6, 6)

    def log_prob(self, action):
        '''
        Target Energy is a energy function that will provide the desired velocity given the current state p(\dot{x} | x)
        We will model it with a gaussian distribution
        '''

        action = action[:, :self.dim]  # torch.Size([1000, 6])
        return self.p_dx.log_prob(action)  # torch.Size([1000])



class TaskGoTO_JoyUpDown(EnergyLeaf):
    '''
        state : 3 part for endeffector
        state[0] : x
        state[1] : dx
        state[2] : joystick Hat Up or down
    '''

    def __init__(self, dim=6, var=None):

        super(TaskGoTO_JoyUpDown, self).__init__()

        self.dim = dim

        self.p_dx = None

    def set_context(self, state):
        '''
        We compute the conditioning variables of our model to have a faster optimization
        '''

        x = state[0]  # Tensor(4, 4), end-effector rotation and position SE(3)
        v = state[1]  # Tensor (1, 6), end-effector spatial velocity V_b

        joystick = state[2]
        # ===========================================#

        # x0 = state[0][0, -1]
        # y0 = state[0][1, -1]

        # k_x=(x0 - 0.3)
        # k_y=(y0 - 0.)

        # dx = -k_x
        # dy = -k_y

        dy = joystick[0][0] * 2.0
        dx = joystick[0][1] * 1.0

        mu = torch.zeros((6))  # Tensor(6, 1)
        mu[0] = dx
        mu[1] = dy

        _var = torch.clone(self.var)

        for i in range(2, 6):
            _var[i, i] = _var[i, i] * 0.1

        self.p_dx = tdist.MultivariateNormal(mu, _var)  # self.var->torch.size(6, 6)

    def log_prob(self, action):
        '''
        Target Energy is a energy function that will provide the desired velocity given the current state p(\dot{x} | x)
        We will model it with a gaussian distribution
        '''

        action = action[:, :self.dim]  # torch.Size([1000, 6])
        return self.p_dx.log_prob(action)  # torch.Size([1000])



class TaskGoTO_JoyFar(EnergyLeaf):

    def __init__(self, dim=6, A=None, b=None, R=None, var=None):
        super(TaskGoTO_JoyFar, self).__init__()
        self.dim = dim

        if A is None:
            A = torch.eye(self.dim).float()
        self.register_buffer('A', A)
        if b is None:
            b = torch.zeros(3).float()
        self.register_buffer('b', b)
        if var is None:
            var = torch.eye(self.dim).float()
        self.register_buffer('var', var)

        if R is None:
            R = torch.eye(3).float()

        R_inv = torch.inverse(R)
        self.register_buffer('R', R)
        self.register_buffer('R_inv', R_inv)

        self.p_dx = None

    def set_context(self, state):
        '''
        We compute the conditioning variables of our model to have a faster optimization
        '''
        x = state[0]  # Tensor(4, 4), end-effector rotation and position SE(3)
        v = state[1]  # Tensor (1, 6), end-effector spatial velocity V_b

        # ===========================================#

        x0 = state[0][0, -1]
        y0 = state[0][1, -1]

        k_x = (x0 - 0.3)
        k_y = (y0 - 0.)

        dx = -k_x
        dy = -k_y

        mu = torch.zeros((6))
        mu[0] = dx
        mu[1] = dy

        _var = torch.clone(self.var)
        for i in range(2, 6):
            _var[i, i] = _var[i, i] * 0.1

        # self.p_dx = tdist.MultivariateNormal(mu, self.var)
        self.p_dx = tdist.MultivariateNormal(mu, _var)

        # print('mu: ', mu)  # Tensor(6, 1)

        # self.p_dx = tdist.MultivariateNormal(mu, self.var)  # self.var->torch.size(6, 6)

    def log_prob(self, action):
        '''
        Target Energy is a energy function that will provide the desired velocity given the current state p(\dot{x} | x)
        We will model it with a gaussian distribution
        '''

        # TODO:
        action = action[:, :self.dim]  # torch.Size([1000, 6])
        return self.p_dx.log_prob(action)  # torch.Size([1000])


class TaskGoTO_JoyClose(EnergyLeaf):

    def __init__(self, dim=6, A=None, b=None, R=None, var=None):
        super(TaskGoTO_JoyClose, self).__init__()
        self.dim = dim

        if A is None:
            A = torch.eye(self.dim).float()
        self.register_buffer('A', A)
        if b is None:
            b = torch.zeros(3).float()
        self.register_buffer('b', b)
        if var is None:
            var = torch.eye(self.dim).float()
        self.register_buffer('var', var)

        if R is None:
            R = torch.eye(3).float()

        R_inv = torch.inverse(R)
        self.register_buffer('R', R)
        self.register_buffer('R_inv', R_inv)

        self.p_dx = None

    def set_context(self, state):
        '''
        We compute the conditioning variables of our model to have a faster optimization
        '''
        x = state[0]  # Tensor(4, 4), end-effector rotation and position SE(3)
        v = state[1]  # Tensor (1, 6), end-effector spatial velocity V_b

        # ===========================================#

        x0 = state[0][0, -1]
        y0 = state[0][1, -1]

        k_x = (x0 - 0.3)
        k_y = (y0 - 0.)

        dx = -k_x
        dy = -k_y

        mu = torch.zeros((6))
        mu[0] = dx
        mu[1] = dy

        _var = torch.clone(self.var)
        for i in range(2, 6):
            _var[i, i] = _var[i, i] * 0.1

        # self.p_dx = tdist.MultivariateNormal(mu, self.var)
        self.p_dx = tdist.MultivariateNormal(mu, _var)

        # print('mu: ', mu)  # Tensor(6, 1)

        # self.p_dx = tdist.MultivariateNormal(mu, self.var)  # self.var->torch.size(6, 6)

    def log_prob(self, action):
        '''
        Target Energy is a energy function that will provide the desired velocity given the current state p(\dot{x} | x)
        We will model it with a gaussian distribution
        '''

        # TODO:
        action = action[:, :self.dim]  # torch.Size([1000, 6])
        return self.p_dx.log_prob(action)  # torch.Size([1000])
