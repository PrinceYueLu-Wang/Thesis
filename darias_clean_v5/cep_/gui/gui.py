from multiprocessing import Process, Lock, Value, Array
from multiprocessing import Queue

import time
from datetime import datetime

import numpy as np

import os
import sys

from tkinter import *


class ProjectGUI(object):

    def __init__(self, lock, axis_array_share, robotstate_array_share=None):
        super(ProjectGUI, self).__init__()

        self.window = Tk()

        self.window.title = "joystick test"

        self.window.geometry('300x300')

        ###############################

        self.lbl_1 = Label(self.window, text="axis 0 with value 0.0")
        self.lbl_2 = Label(self.window, text="axis 1 with value 0.0")

        # row 0 to row 1
        self.lbl_1.grid(column=0, row=0)
        self.lbl_2.grid(column=0, row=1)

        ###############################

        self.info_q = [Label(self.window, text = "joint {} with value 0.0 ".format(x)) for x in range(7)]


        # row 2 to row 8 
        for i in range(7):
            self.info_q[i].grid(column=0, row =2+i)
        
        self.eef_p_dict=["x","y","z"]
        self.eef_v_dict=["dx","dy","dz","row","pitch","yaw"]
        
        self.info_eef_pos = [Label(self.window, text = "eef position {} with value 0.0 ".format(self.eef_p_dict[x])) for x in range(3)]
        self.info_eef_vel = [Label(self.window, text = "eef velocity {} with value 0.0 ".format(self.eef_v_dict[x])) for x in range(6)]

        # row 9 to row 11 
        for i in range(3):
            self.info_eef_pos[i].grid(column=0, row =9+i)
        
        # row 12 to row 17 
        for i in range(6):
            self.info_eef_vel[i].grid(column=0, row =12+i)

        ###############################

        self.lock = lock

        self.lock_enable = True

        self.robotstate_enable = True

        ###############################

        self.axis = axis_array_share

        if robotstate_array_share is None:

            self.robotstate = np.zeros(16)

        else :

            self.robotstate = robotstate_array_share

        ###############################
        self.window.after(200, self.refresh)
        

    def refresh(self):

        if self.lock_enable:

            self.lock.acquire()

        a = self.axis[0]
        b = self.axis[1]

        a = round(a, 3)
        b = round(b, 3)

        robotinfo = [self.robotstate[x] for x in range(16)]

        if self.lock_enable:

            self.lock.release()

        self.lbl_1['text'] = "axis 0 (y direction) with value {}".format(a)
        self.lbl_2['text'] = "axis 1 (x direction) with value {}".format(b)

        q = robotinfo[0:7]
        x = robotinfo[7:10]
        v = robotinfo[10:16]

        for i in range(7):
             self.info_q[i]['text'] = "joint {} with value {} ".format(i,q[i])
        
        for i in range(3):
             self.info_eef_pos[i]['text'] = "eef position {} with value {} ".format(self.eef_p_dict[i],x[i])
        
        for i in range(6):
             self.info_eef_vel[i]['text'] = "eef velocity {} with value {} ".format(self.eef_v_dict[i],v[i])



        self.window.after(200, self.refresh)

