import os
import sys

dir=os.path.dirname(__file__)
rootlib=os.path.abspath(os.path.join(dir,'..'))
sys.path.append(rootlib)

from cep_ import DariasHandSimple

import numpy as np
import pybullet as p
import time

import tkinter as tk


from cep_.envs import DariasHandSimple
from cep_.cep_models import cep_simple_model

import torch


joint_limit_buffers = 0.01
joint_limits = np.array([2.96, 2.09, 2.96, 2.09, 2.96, 2.09, 2.96]) - joint_limit_buffers

device = torch.device('cpu')

class CEPPolicy():
    def __init__(self, dt=1 / 240., dtype='float64'):
        self.dt = dt
        self.dtype = dtype

        self.controller = cep_simple_model()

    def policy(self, state):
        joint_poses = state[0,0:7]
        joint_vels = state[0,7:]

        action = self.controller.policy(state)

        x,dx = self.step(joint_poses, joint_vels, action)
        return x, dx


    def step(self, joint_poses, joint_vels, joint_accs):
        joint_poses = joint_poses + joint_vels * self.dt
        joint_vels = joint_vels + joint_accs * self.dt
        return joint_poses, joint_vels


def experiment():
    '''
    envlist:[it can take values [1,2,3]. It provides the type of environment we are using]
    results_dir: path to the folder in which we are saving the results
    '''

    time_step = 1 / 250.

    env = DariasHandSimple(time_step=time_step)

    policy = CEPPolicy(dt=time_step)
    ################

    n_trials = 100
    horizon = 1500
    c = 0
    s = 0
    ################
    q_initset=[ 0.41218702,  0.96820871, -2.35509904,  1.6827199 , -0.88989564,
                      -0.34380408,  1.58004667,  0.01671192, -0.02585537, -0.05823977,
                       0.02063612,  0.10688131,  0.04823189, -0.08189419]
    q_initset=q_initset[:7]
    ################


    for itr in range(n_trials):
        print('Iteration: {}'.format(itr))
        # state = env.reset()
        state = env.reset(q0=q_initset)
        p.addUserDebugLine([0., 0., -0.189], [1.5, 0., -0.189], [1., 0., 0.])
 
        for i in range(horizon):
            init = time.time()

            #### Get Control Action (Position Control)####
            a = policy.policy(state)
            state, reward, done, success = env.step(a)
            #############################

            end = time.time()
            time.sleep(np.clip(time_step - (end - init), 0, time_step))

    p.disconnect()


if __name__ == '__main__':
    p.connect(p.GUI_SERVER, 1234,
              options='--background_color_red=1. --background_color_green=1. --background_color_blue=1.')
    p.resetDebugVisualizerCamera(2.2, 55.6, -47.4, [0.04, 0.06, 0.31])

    # TK_GUI()
    experiment()
