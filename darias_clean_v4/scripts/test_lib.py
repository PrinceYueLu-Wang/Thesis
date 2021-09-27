import os
import sys

import numpy as np
import pybullet as p
import time

import torch

dir = os.path.dirname(__file__)
rootlib = os.path.abspath(os.path.join(dir, '..'))
sys.path.append(rootlib)

from cep_.envs import DariasHandSimple
from cep_.cep_models import cep_simple_model
from cep_.cep_models import cep_models_joy
from cep_.joystick import Joystick

joint_limit_buffers = 0.01
joint_limits = np.array([2.96, 2.09, 2.96, 2.09, 2.96, 2.09, 2.96]) - joint_limit_buffers

device = torch.device('cpu')

from multiprocessing import Process, Value, Lock, Array


class JOYPolicy:

    def __init__(self, dt=1 / 240., dtype='float64'):
        self.dt = dt
        self.dtype = dtype

        self.controller = cep_models_joy()
        # self.controller = cep_simple_model()

    def policy(self, state):
        joint_poses = state[0, 0:7]
        joint_vels = state[0, 7:]

        action = self.controller.policy(state)

        ## Smoothing
        alpha = 0.1
        action = alpha * action + (1 - alpha) * joint_vels

        x, dx = self.step(joint_poses, joint_vels, action)
        return x, dx

    def step(self, joint_poses, joint_vels, joint_vels2):
        joint_poses = joint_poses + joint_vels2 * self.dt
        joint_vels = joint_vels2
        return joint_poses, joint_vels


class Experiment(Process):

    def __init__(self, lock, axis_array_share):
        super(Experiment, self).__init__()

        self.time_step = 1 / 250.

        self.env = DariasHandSimple(time_step=self.time_step)

        self.policy = JOYPolicy(dt=self.time_step)

        ################

        self.n_trials = 100
        self.horizon = 1500

        ################

        self.lock = lock
        self.axis_array_share = axis_array_share
        self.axis_array = np.zeros(shape=(1, 2))

    def EnvInit(self):
        # return q ndarray(1,14)
        q_InitSet = [0.41218702, 0.96820871, -2.35509904, 1.6827199, -0.88989564,
                     -0.34380408, 1.58004667, 0.01671192, -0.02585537, -0.05823977,
                     0.02063612, 0.10688131, 0.04823189, -0.08189419]

        # joint 0 to 6 : right arm
        q_InitSet = q_InitSet[:7]

        return self.env.reset(q0=q_InitSet)

    def GetJoyStick(self):

        # return joy ndarray(1,2)

        self.lock.acquire()
        self.axis_array[0][0] = round(self.axis_array_share[0], 2)
        self.axis_array[0][1] = round(self.axis_array_share[0], 2)

        self.lock.release()

    def StateCat(self, state_env, state_joy):

        # return ndarray(1,16) 14 + 2

        return np.concatenate((state_env, state_joy), axis=1)

    def run(self):

        for iter_trial in range(self.n_trials):
            print("Iteration No.{} begins".format(iter_trial))

            ###############################

            state_env = self.EnvInit()
            self.GetJoyStick()
            state = self.StateCat(state_env=state_env, state_joy=self.axis_array)

            ###############################

            for i in range(self.horizon):
                init = time.time()

                ###############################

                action = self.policy.policy(state)
                state, reward, done, success = self.env.step(action)

                #############################

                end = time.time()
                time.sleep(np.clip(self.time_step - (end - init), 0, self.time_step))

        p.disconnect()


if __name__ == '__main__':
    #####################################
    p.connect(p.GUI_SERVER, 1234,
              options='--background_color_red=1. --background_color_green=1. --background_color_blue=1.')
    p.resetDebugVisualizerCamera(2.2, 55.6, -47.4, [0.04, 0.06, 0.31])

    #####################################

    multiprocess_lock = Lock()
    multiprocess_joyarray = Array('f', [0.0, 0.0])

    joy_controller = Joystick(lock=multiprocess_lock, axis_array_share=multiprocess_joyarray)
    experiment = Experiment(lock=multiprocess_lock, axis_array_share=multiprocess_joyarray)

    joy_controller.start()
    experiment.start()

    experiment.join()
    joy_controller.join()

    #####################################
