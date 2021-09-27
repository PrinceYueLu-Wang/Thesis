import torch
import torch.distributions as tdist
import numpy as np

from .energy_leaf import EnergyLeaf

from cep_.utils import eul2rot, rot2eul, rot2quat
from cep_.liegroups.torch import SO3, SE3


class TaskGoToLeaf(EnergyLeaf):
    def __init__(self, dim=6, A = None, b = None, R = None, var=None):
        super(TaskGoToLeaf, self).__init__()
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

        ## variables for computation ##
        self.p_dx = None

    def set_context(self, state):
        '''
        We compute the conditioning variables of our model to have a faster optimization
        '''
        x = state[0]  # Tensor(4, 4), end-effector rotation and position SE(3)
        v = state[1]  # Tensor (1, 6), end-effector spatial velocity V_b
        #print('state:', state)
        # print('x: ', x)
        # print('v: ', v)

        #print('self.R_inv: ', self.R_inv) # Tensor (4, 4)
        #print('R: ', self.R) # Tensor (4, 4)

        Htl = torch.matmul(self.R_inv, x) # R_inv * X
        #print('Htl: ', Htl)
        Xe = SE3.from_matrix(Htl) # <cep.liegroups.torch.se3.SE3Matrix>, SE(3)
        #print('Xe: ', Xe)
        xtl = Xe.log() # Tensor(1, 6), (omega, V)
        #print('xtl: ', xtl)
        vtl = -xtl
        A = SE3.from_matrix(self.R) # <cep.liegroups.torch.se3.SE3Matrix>, SE(3), R
        #print('A: ', A)
        Adj_lw = A.adjoint() # Adjoint map (Spatial velocity from one frame to another frame), Tensor (6,6),
        #print('Adj_lw: ', Adj_lw)
        ve_w = torch.matmul(Adj_lw, vtl) # Tensor(6, 1)
        #print('v_ew: ', ve_w)
        ###########################################

        scale = 60.
        mu = scale * ve_w - 1.2 * scale * v
        #print('mu: ', mu)  # Tensor(6, 1)

        self.p_dx = tdist.MultivariateNormal(mu, self.var)  # self.var->torch.size(6, 6)

    def log_prob(self, action):
        '''
        Target Energy is a energy function that will provide the desired velocity given the current state p(\dot{x} | x)
        We will model it with a gaussian distribution
        '''

        # TODO:
        action = action[:, :self.dim]  # torch.Size([1000, 6])
        return self.p_dx.log_prob(action)  # torch.Size([1000])


class TaskJoyStickLeaf(EnergyLeaf):

    def __init__(self, dim=6, A = None, b = None, R = None, var=None):
        super(TaskJoyStickLeaf, self).__init__()
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

        ## variables for computation ##
        self.p_dx = None

    def set_context(self, state):
        '''
        We compute the conditioning variables of our model to have a faster optimization
        '''
        x = state[0]  # Tensor(4, 4), end-effector rotation and position SE(3)
        v = state[1]  # Tensor (1, 6), end-effector spatial velocity V_b

        #===========================================#
        x0 = state[0][0, -1]
        y0 = state[0][1, -1]

        k_x=(x0 - 0.3)
        k_y=(y0 - 0.)

        dx = -k_x
        dy = -k_y

        mu=torch.zeros((6))
        mu[0]=dx
        mu[1]=dy

        _var=torch.clone(self.var)
        for i in range(2,6):
            _var[i,i]=_var[i,i]*0.1

        # self.p_dx = tdist.MultivariateNormal(mu, self.var)
        self.p_dx = tdist.MultivariateNormal(mu, _var)


        #print('mu: ', mu)  # Tensor(6, 1)

        # self.p_dx = tdist.MultivariateNormal(mu, self.var)  # self.var->torch.size(6, 6)

    def log_prob(self, action):
        '''
        Target Energy is a energy function that will provide the desired velocity given the current state p(\dot{x} | x)
        We will model it with a gaussian distribution
        '''

        # TODO:
        action = action[:, :self.dim]  # torch.Size([1000, 6])
        return self.p_dx.log_prob(action)  # torch.Size([1000])