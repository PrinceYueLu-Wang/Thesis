from multiprocessing import Process, Lock, Value, Array
from multiprocessing import Queue

import time
from datetime import datetime

import os
import sys

from tkinter import *


class ProjectGUI(object):

    def __init__(self, lock, axis_array_share):
        super(ProjectGUI, self).__init__()

        self.window = Tk()

        self.window.title = "joystick test"

        self.window.geometry('200x200')

        ###############################

        self.lbl_1 = Label(self.window, text="axis 0 with value 0.0")
        self.lbl_2 = Label(self.window, text="axis 1 with value 0.0")

        self.lbl_1.grid(column=0, row=0)
        self.lbl_2.grid(column=0, row=1)

        ###############################

        self.lock = lock
        self.axis = axis_array_share

        ###############################

        self.window.after(200, self.refresh)

        ###############################

        self.lock_enable = False

    def refresh(self):

        if self.lock_enable:
            self.lock.acquire()

        a = self.axis[0]
        b = self.axis[1]

        a = round(a, 3)
        b = round(b, 3)

        self.lbl_1['text'] = "axis 0 with value {}".format(a)
        self.lbl_2['text'] = "axis 0 with value {}".format(b)

        if self.lock_enable:
            self.lock.release()

        self.window.after(200, self.refresh)

