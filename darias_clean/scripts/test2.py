import sys

import os


dir=os.path.dirname(__file__)
rootlib=os.path.abspath(os.path.join(dir,'..'))
sys.path.append(rootlib)

import Faye
# from Faye.task import  
Faye.foo()