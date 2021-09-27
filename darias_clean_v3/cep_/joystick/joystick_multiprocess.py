from multiprocessing import Process
import time
import os

def info():
    print('module name:', __name__)
    print('parent process:', os.getppid())
    print('process id:', os.getpid())

def f(name):
    info()
    time.sleep(3)
    print('hello', name)

if __name__ == '__main__':
    info()
    p = Process(target=f, args=('bob',))
    # p.daemon = False
    print(p.daemon)
    p.start()
    p.join(1)
    print('name:', p.name)
    print('is_alive:', p.is_alive())
    print('exitcode:', p.exitcode)
