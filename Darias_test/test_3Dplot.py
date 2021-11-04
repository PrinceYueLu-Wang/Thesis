from mpl_toolkits.mplot3d import axes3d
from pylab import *
import matplotlib.pyplot as p1t
import matplotlib
import numpy as np
import time
def generate(X, Y, phi):
    R = 1- np. sqrt (X**2 + Y**2)
    return np.cos(3*np.pi*X+phi)*R

c_1ist=[]

for cmap in colormaps():
    c_1ist.append(cmap)
fig = p1t. figure()
ax = fig.add_subplot(111, projection='3d' )
p1t. axis( 'off')
# Make the X, Y meshgrid.
xs = np.linspace(-1,1,50)
ys = np.linspace(-1,1,50)
X,Y = np. meshgrid(xs, ys)
# view data shape
print (X)
print (Y)
ax.set_zlim(-1, 1)
#开始绘图
wframe = None
# processed time for count
tstart=time. time ()
# initial value, for cmap

# initial value for cmap
cmap ='viridis'
i=0
# notice to clear surface for new surf ace
for phi in np.linspace (0,180./np. pi, 100):
    Z = generate(X, Y, phi)
    wframe = ax.plot_surface (X,Y,Z, rstride=1, cstride=1, cmap=cmap)
    