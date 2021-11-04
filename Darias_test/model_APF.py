# %matplotlib inline

import matplotlib
import numpy as np
import matplotlib.pyplot as plt

xs = np.linspace(-1,1,50)
ys = np.linspace(-1,1,50)
zs= np.linspace(-1,1,50)
X,Y,Z= np. meshgrid(xs, ys,zs)
num=len(xs)

APF=np.zeros(shape=(num,num,num))

for i in range(0,num):
    for j in range(0,num):
        for k in range(0,num):
            APF[i][j][k]=0.5*((X[i][j][k])**2+0.5*(Y[i][j][k])**2+0.5*(Z[i][j][k])**2)

gradAPF=np.gradient(APF,0.02,0.02,0.02,edge_order=1)

gradAPF_x=gradAPF[0]
gradAPF_y=gradAPF[1]
gradAPF_z=gradAPF[2]


u=-gradAPF_x[::10,::10,::10]
v=-gradAPF_y[::10,::10,::10]
w=-gradAPF_z[::10,::10,::10]

x0=X[::10,::10,::10]
y0=Y[::10,::10,::10]
z0=Z[::10,::10,::10]

fig = plt.figure()
ax = fig.gca(projection='3d')

ax.set_xlim(-1,1)
ax.set_ylim(-1,1)
ax.set_zlim(-1,1)

ax.quiver(x0, y0, z0, u, v, w, length=0.1, color = 'black')

plt.show()
