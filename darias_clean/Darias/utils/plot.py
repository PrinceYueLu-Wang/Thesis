import matplotlib.pyplot as plt
import numpy as np
class PlotData():

    def __init__(self):
        self.q=[]
        self.dq=[]
        self.ddq=[]
        self.x_EEF_world=[]
        self.v_EEF_world=[]
        self.time=[]
        self.idxEEF=7

    def DataUpdate(self,q=None,dq=None,ddq=None,x_EEF_world=None,v_EEF_world=None,time=None):

        self.q.append(q)
        self.dq.append(dq)
        self.ddq.append(ddq)
        self.x_EEF_world.append(x_EEF_world)
        self.v_EEF_world.append(v_EEF_world)
        self.time.append(time)

    def Plot_q(self,allJoints=False):

        fig = plt.figure(1)

        if allJoints==False:
            temp_q=[x[self.idxEEF-1] for x in self.q]
            plt.plot(self.time,temp_q)
            plt.show()
        elif allJoints==True:
            temp_q=[]
            for i in range(0,7):
                temp_q.append([x[i] for x in self.q])
            for i in range(0,7):
                temp_ax=fig.add_subplot(7,1,i+1)
                plt.plot(self.time,temp_q[i])
            plt.show()

    def Plot_dq(self,allJoints=False):

        fig = plt.figure(1)

        if allJoints==False:
            temp_dq=[x[self.idxEEF-1] for x in self.dq]
            plt.plot(self.time,temp_dq)
            plt.show()
        elif allJoints==True:
            temp_dq=[]
            for i in range(0,7):
                temp_dq.append([x[i] for x in self.dq])
            for i in range(0,7):
                temp_ax=fig.add_subplot(7,1,i+1)
                plt.plot(self.time,temp_dq[i])
            plt.show()

    def Plot_x_EEF_world(self):
        
        fig = plt.figure(1)
        ax = plt.axes(projection='3d')
        ax.set_xlabel('x')
        ax.set_ylabel('y')
        ax.set_zlabel('z')


        data_x=[x[0:3,-1][0] for x in self.x_EEF_world]
        data_y=[x[0:3,-1][1] for x in self.x_EEF_world]
        data_z=[x[0:3,-1][2] for x in self.x_EEF_world]

        # ax.set_xlim(np.min(data_x),np.max(data_x))
        # ax.set_ylim(np.min(data_y),np.max(data_y))
        # ax.set_zlim(np.min(data_z),np.max(data_z))

        # ax.plot3D(data_x, data_y, data_z, 'red')

        ax.scatter3D(
            data_x[::1]    ,
            data_y[::1]    , 
            data_z[::1]    , 
            # c=self.time[::10]  , 
            # cmap='Reds'
            )

        plt.show()

    def Plot_ddq(self,allJoints=False):

        fig = plt.figure(1)

        if allJoints==False:
            temp_ddq=[x[self.idxEEF-1] for x in self.ddq]
            plt.plot(self.time,temp_ddq)
            plt.show()
        elif allJoints==True:
            temp_ddq=[]
            for i in range(0,7):
                temp_ddq.append([x[i] for x in self.ddq])

            for i in range(0,7):
                temp_ax=fig.add_subplot(7,1,i+1)
                temp_ax.set_ylim(np.np.min(temp_ddq[i]),np.np.max(temp_ddq[i]))
                plt.plot(self.time,temp_ddq[i])
            plt.show()

    def Show(self,q=None,dq=None,ddq=None,x_EEF_world=None):
        pass