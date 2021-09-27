import pygame

from multiprocessing import Process, Lock, Value, Array
from multiprocessing import Queue
import time
from datetime import datetime
import os
import sys

import queue

from tkinter import *


# class Joystick(Process):
#     def __init__(self, lock, axis_0, axis_1):
#         super(Joystick, self).__init__()
#
#         self.lock = lock
#         self.axis_0 = axis_0
#         self.axis_1 = axis_1
#
#     def run(self):
#         pygame.init()
#
#         joystick = pygame.joystick.Joystick(0)
#         joystick.init()
#
#         while True:
#             pygame.event.pump()
#
#             self.lock.acquire()
#
#             self.axis_0.value = joystick.get_axis(0)
#             self.axis_1.value = joystick.get_axis(1)
#
#             self.lock.release()
#
#             # axis = [axis_0, axis_1]
#
#             # self.q.put(axis, data = c_queue.get())
#             # print(axis_0)
#             # print(self.q.qsize())
#             # print("input queue")
#
#             time.sleep(0.01)
#
#
# # class StateGUI(Process):
# #
# #     def __init__(self, inputqueue):
# #         super(StateGUI, self).__init__()
# #
# #         self.queue = inputqueue
# #
# #         self.window = Tk()
# #
# #         self.window.title = "joystick test"
# #
# #         self.window.geometry('200x200')
# #
# #         self.lbl_1 = Label(self.window, text="axis 0 with value 0.0")
# #         self.lbl_2 = Label(self.window, text="axis 1 with value 0.0")
# #
# #         # self.txt_1 = Entry(self.window, width=10)
# #         # self.txt_2 = Entry(self.window, width=10)
# #         #
# #         self.lbl_1.grid(column=0, row=0)
# #         self.lbl_2.grid(column=0, row=1)
# #
# #         self.window.mainloop()
# #
# #         print("gui init done!")
# #         #
# #         # self.txt_1.grid(column=1, row=0)
# #         # self.txt_2.grid(column=1, row=1)
# #         #
# #         # self.txt_1.insert(str(0.00))
# #         # self.txt_1.insert(str(0.00))
# #
# #     def run(self):
# #
# #         self.window.after(200, self.refresh())
# #
# #     def refresh(self):
# #         if not self.queue.empty():
# #             data = self.queue.get()
# #
# #             # self.txt_1.insert(str(data[0]))
# #             # self.txt_2.insert((data[1]))
# #
# #             self.lbl_1['text'] = "axis 0 with value {}".format(data[0])
# #             self.lbl_2['text'] = "axis 0 with value {}".format(data[1])
# #         else:
# #             now = datetime.now()
# #             self.lbl_1['text'] = "axis 0 with value {}".format(now.strftime("%H:%M:%S"))
# #             self.lbl_2['text'] = "axis 0 with value {}".format(now.strftime("%H:%M:%S"))
#
#
# class StateGUI(object):
#
#     def __init__(self, lock, axis_0, axis_1):
#         super(StateGUI, self).__init__()
#
#         self.window = Tk()
#
#         self.window.title = "joystick test"
#
#         self.window.geometry('200x200')
#
#         self.lbl_1 = Label(self.window, text="axis 0 with value 0.0")
#         self.lbl_2 = Label(self.window, text="axis 1 with value 0.0")
#
#         self.lbl_1.grid(column=0, row=0)
#         self.lbl_2.grid(column=0, row=1)
#
#         self.lock = lock
#         self.axis_0 = axis_0
#         self.axis_1 = axis_1
#
#         # self.window.mainloop()
#
#         # print("gui init done!")
#
#         self.window.after(200, self.refresh)
#
#     def refresh(self):
#         # data = c_queue.get(block=False)
#         # print(data)
#         # if not c_queue.empty():
#         #     data = c_queue.get(timeout=1)
#         #
#         #     self.lbl_1['text'] = "axis 0 with value {}".format(data[0])
#         #     self.lbl_2['text'] = "axis 0 with value {}".format(data[1])
#         # else:
#         #     now = datetime.now()
#         #     print(now.strftime("%H:%M:%S"))
#         #     self.lbl_1['text'] = "axis 0 with value {}".format(now.strftime("%H:%M:%S"))
#         #     self.lbl_2['text'] = "axis 0 with value {}".format(now.strftime("%H:%M:%S"))
#         self.lock.acquire()
#
#         self.lbl_1['text'] = "axis 0 with value {}".format(self.axis_0.value)
#         self.lbl_2['text'] = "axis 0 with value {}".format(self.axis_1.value)
#
#         self.lock.release()
#
#         self.window.after(200, self.refresh)
#
#
# if __name__ == "__main__":
#
#     # q1 = queue.LifoQueue(1)
#     # q1 = Queue.LifoQueue
#
#     lock1 = Lock()
#     axis0 = Value('f', 0.0)
#     axis1 = Value('f', 0.0)
#
#     joy = Joystick(lock1, axis0, axis1)
#
#     gui = StateGUI(lock1, axis0, axis1)
#
#     joy.start()
#
#     gui.window.mainloop()
#
#     joy.join()

class Joystick(Process):
    def __init__(self, lock, axis_array_share):
        super(Joystick, self).__init__()

        self.lock = lock
        self.axis = axis_array_share

    def run(self):
        pygame.init()

        joystick = pygame.joystick.Joystick(0)
        joystick.init()

        while True:
            pygame.event.pump()

            self.lock.acquire()

            self.axis[0] = joystick.get_axis(0)
            self.axis[1] = joystick.get_axis(1)

            self.lock.release()

            time.sleep(0.01)

            
class StateGUI(object):

    def __init__(self, lock, axis_array_share):
        super(StateGUI, self).__init__()

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

    def refresh(self):

        self.lock.acquire()

        a=self.axis[0]
        b=self.axis[1]

        a = round(a, 2)
        b = round(b, 2)

        self.lbl_1['text'] = "axis 0 with value {}".format(a)
        self.lbl_2['text'] = "axis 0 with value {}".format(b)

        self.lock.release()

        self.window.after(200, self.refresh)


if __name__ == "__main__":

    # q1 = queue.LifoQueue(1)
    # q1 = Queue.LifoQueue

    lock0 = Lock()
    axis_array_share = Array('d', [0., 0.])

    joy = Joystick(lock0, axis_array_share)

    gui = StateGUI(lock0, axis_array_share)

    joy.start()

    gui.window.mainloop()

    joy.join()
