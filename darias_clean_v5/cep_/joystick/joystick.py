import pygame

from multiprocessing import Process, Lock, Value

import time

from tkinter import *


class Joystick(Process):
    def __init__(self, lock, axis_0, axis_1):
        super(Joystick, self).__init__()

        self.lock = lock
        self.axis_0 = axis_0
        self.axis_1 = axis_1

    def run(self):
        pygame.init()

        joystick = pygame.joystick.Joystick(0)
        joystick.init()

        while True:
            pygame.event.pump()

            self.lock.acquire()

            self.axis_0.value = joystick.get_axis(0)
            self.axis_1.value = joystick.get_axis(1)

            self.lock.release()



            time.sleep(0.01)


class StateGUI(object):

    def __init__(self, lock, axis_0, axis_1):
        super(StateGUI, self).__init__()

        self.window = Tk()

        self.window.title = "joystick test"

        self.window.geometry('200x200')

        self.lbl_1 = Label(self.window, text="axis 0 with value 0.0")
        self.lbl_2 = Label(self.window, text="axis 1 with value 0.0")

        self.lbl_1.grid(column=0, row=0)
        self.lbl_2.grid(column=0, row=1)

        self.lock = lock
        self.axis_0 = axis_0
        self.axis_1 = axis_1

        # self.window.mainloop()

        # print("gui init done!")

        self.window.after(200, self.refresh)

    def refresh(self):

        self.lock.acquire()

        self.lbl_1['text'] = "axis 0 with value {}".format(self.axis_0.value)
        self.lbl_2['text'] = "axis 0 with value {}".format(self.axis_1.value)

        self.lock.release()

        self.window.after(200, self.refresh)


if __name__ == "__main__":

    # q1 = queue.LifoQueue(1)
    # q1 = Queue.LifoQueue

    lock1 = Lock()
    axis0 = Value('f', 0.0)
    axis1 = Value('f', 0.0)

    axis

    joy = Joystick(lock1, axis0, axis1)

    gui = StateGUI(lock1, axis0, axis1)

    joy.start()

    gui.window.mainloop()

    joy.join()

