import torch.nn as nn
from  torch.linalg import norm


class EnergyLeaf(nn.Module):
    '''
    An Energy Leaf is an Energy Base Model that provides the unnormalized log prob(action|state)

    state : list of tensor 

    '''

    def __init__(self):
        super(EnergyLeaf, self).__init__()

    def set_context(self, state):
        pass

    def log_prob(self, action):
        pass


    def set_var(self,cur_pos, target_pos, threshold):
        pass


