
import os
import sys

import numpy as np
import pybullet as p
import time

from multiprocessing import Process, Value, Lock, Array

import torch

dir = os.path.dirname(__file__)
rootlib = os.path.abspath(os.path.join(dir, '..'))
sys.path.append(rootlib)

from cep_.envs import DariasHandSimple
from cep_.cep_models import cep_models_joy
from cep_.cep_models import cep_models_joy_3leaf


from cep_.joystick import Joystick
from cep_.gui import ProjectGUI

from cep_.kinematics import DarIASArm

joint_limit_buffers = 0.01
joint_limits = np.array([2.96, 2.09, 2.96, 2.09, 2.96, 2.09, 2.96]) - joint_limit_buffers

device = torch.device('cpu')

GUI_ENABLE = True

class JOYPolicy:

    def __init__(self, dt=1 / 240., dtype='float64'):
        self.dt = dt
        self.dtype = dtype
        
        # self.controller = cep_simple_model()
        # self.controller = cep_models_joy()
        # self.controller = cep_models_joy_3leaf()

        self.controller = cep_models_joy_3leaf()

    def policy(self, state):

        # state torch.Size(1,16)

        joint_poses = state[0, 0:7]
        joint_vels = state[0, 7:14]

        action = self.controller.policy(state)

        # smoothing
        alpha = 0.1
        action = alpha * action + (1 - alpha) * joint_vels

        x, dx = self.step(joint_poses, joint_vels, action)
        return x, dx

    def step(self, joint_poses, joint_vels, joint_vels2):

        joint_poses = joint_poses + joint_vels2 * self.dt
        joint_vels = joint_vels2

        return joint_poses, joint_vels


class Experiment(Process):

    def __init__(self, lock, axis_array_share, robotstate_array_share=None):
        super(Experiment, self).__init__()

        self.time_step = 1 / 250.

        self.env = DariasHandSimple(time_step=self.time_step)

        self.policy = JOYPolicy(dt=self.time_step)

        ################

        self.n_trials = 100
        self.horizon = 1500

        ################


        self.lock = Lock()

        self.axis_array_share = axis_array_share
        self.axis_array = np.zeros(shape=(1, 2))

        self.robotstate_array_share = robotstate_array_share
        self.robotstate_array = np.zeros(shape=(1, 16)) # 7 + 3 + 6

        ################

        self.pseudo = DarIASArm()

        self.refresh_timecounter = 0

    def EnvInit(self):
        # return q ndarray(1,14)

        # q_InitSet = [0.41218702, 0.96820871, -2.35509904, 1.6827199, -0.88989564,
        #              -0.34380408, 1.58004667, 0.01671192, -0.02585537, -0.05823977,
        #              0.02063612, 0.10688131, 0.04823189, -0.08189419]

        # joint 0 to 6 : right arm
        # q_InitSet = q_InitSet[:7]

        q_InitSet = [-0.3917, -0.0236, -1.7561, 1.5493, 0.3208, -1.1331, 1.1331]
        # q_InitSet = [-0.17590232 ,-2.00404716 , 1.14864648 ,-0.89694388 , 0.57499068 , 2.17113403 ,-1.08167996]

        return self.env.reset(q0=q_InitSet)

    def GetJoyStick(self):

        # return joy ndarray(1,2)

        self.lock.acquire()
        
        a = round(self.axis_array_share[0], 2)
        b = round(self.axis_array_share[1], 2)

        if abs(a) < 0.1:
            a = 0
        if abs(b) < 0.1:
            b = 0

        self.axis_array[0][0] = a
        self.axis_array[0][1] = b


        # self.axis_array[0][0] = round(self.axis_array_share[0], 2)
        # self.axis_array[0][1] = round(self.axis_array_share[1], 2)

        self.lock.release()

    def StateCat(self, state_env, state_joy):

        # return ndarray(1,16) 14 + 2

        return np.concatenate((state_env, state_joy), axis=1)

    def PseudoUpdate(self, state):
        
        q = state[0] # np.shape (7)
        dq = state[1] # np.shape (7)

        self.pseudo.update_kindyn(q)

        J = self.pseudo.eef_worldJ()  # np.shape (6,7)

        x = self.pseudo.eef_worldPos() # np.shape (3)
        v = np.matmul(J, dq) # np.shape (6)

        self.robotstate_array = np.concatenate((q,x,v)) #np shape (16,)

        self.lock.acquire()

        for i in range(16):

            self.robotstate_array_share[i] =  self.robotstate_array[i]

        self.lock.release()

        return 0


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

                # action : tuple (q, dq)    
                action = self.policy.policy(state)

                state, reward, done, success = self.env.step(action)

                ###############################

                # ! robot state to GUI
                # ! too frequent lock.acquire -> lag
                # ! 5 times refresh per second

                if GUI_ENABLE:

                    self.refresh_timecounter += 1 

                    if self.refresh_timecounter == 5:

                        self.PseudoUpdate(action)
                        self.refresh_timecounter = 0

                ###############################

                # ! here should concatenate state(1,14) with joystick(1,2)
                self.GetJoyStick()
                state = self.StateCat(state_env=state,state_joy=self.axis_array)

                #############################

                end = time.time()
                print(end-init)

                time.sleep(np.clip(self.time_step - (end - init), 0, self.time_step))

        p.disconnect()


if __name__ == "__main__":
    #####################################
    p.connect(p.GUI_SERVER, 1234,
              options='--background_color_red=1. --background_color_green=1. --background_color_blue=1.')
    p.resetDebugVisualizerCamera(2.2, 55.6, -47.4, [0.04, 0.06, 0.31])

    #####################################


    multiprocess_lock = Lock()
    multiprocess_joyarray = Array('f', [0.0, 0.0])
    multiprocess_robotarray = Array('f', [0.0] * 16)

    joy_controller = Joystick(lock=multiprocess_lock, axis_array_share=multiprocess_joyarray)
    
    # experiment = Experiment(lock=multiprocess_lock, axis_array_share=multiprocess_joyarray)
    
    experiment = Experiment(lock=multiprocess_lock, 
                            axis_array_share=multiprocess_joyarray,
                            robotstate_array_share=multiprocess_robotarray)


    if GUI_ENABLE:
        # gui = ProjectGUI(lock=multiprocess_lock, axis_array_share=multiprocess_joyarray)
        gui = ProjectGUI(lock=multiprocess_lock, 
                        axis_array_share=multiprocess_joyarray,
                        robotstate_array_share=multiprocess_robotarray)

    joy_controller.start()
    experiment.start()

    if GUI_ENABLE:
        gui.window.mainloop()

    experiment.join()
    joy_controller.join()

    #####################################
