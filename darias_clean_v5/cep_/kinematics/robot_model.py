import pinocchio as pin
import os
import time
import numpy as np


class Robot(object):
    def __init__(self, urdf_file, link_name_list):
        self.model = pin.buildModelFromUrdf(urdf_file)
        self.data = self.model.createData()

        self.links_names = link_name_list
        self.links_ids = [
            self.model.getFrameId(link_name)
            for link_name in self.links_names
        ]

        self.eef_id = self.links_ids[-1]

    def update_kindyn(self, q):
        pin.computeJointJacobians(self.model, self.data, q)
        pin.framesForwardKinematics(self.model, self.data, q)

    def links_fk(self, rotation=False):
        if rotation:
            H = []
            for link_id in self.links_ids:
                Hi = np.eye(4)
                Hi[:3, :3] = np.asarray(self.data.oMf[link_id].rotation)
                Hi[:3, -1] = np.asarray(self.data.oMf[link_id].translation)
                H.append(Hi)
            return H
        else:
            return [
                np.asarray(self.data.oMf[link_id].translation).reshape(-1).tolist()
                for link_id in self.links_ids
            ]

    def links_J(self):
        J_list = []
        for link_id in self.links_ids:
            Ji = pin.getFrameJacobian(
                self.model,
                self.data,
                link_id,
                pin.ReferenceFrame.WORLD,
            )
            J_list.append(Ji)
        return J_list

    def link_fk(self, name_idx):
        return [
            np.asarray(self.data.oMf[self.model.getFrameId(name_idx)].translation).reshape(-1).tolist(),
            np.asarray(self.data.oMf[self.model.getFrameId(name_idx)].rotation)
        ]

    def link_J(self, name_idx):
        return pin.getFrameJacobian(
                self.model,
                self.data,
                self.model.getFrameId(name_idx),
                pin.ReferenceFrame.LOCAL_WORLD_ALIGNED
            )
    def link_localJ(self, name_idx):
        return pin.getFrameJacobian(
                self.model,
                self.data,
                self.model.getFrameId(name_idx),
                pin.ReferenceFrame.LOCAL
            )

    def link_worldJ(self, name_idx):
        return pin.getFrameJacobian(
                self.model,
                self.data,
                self.model.getFrameId(name_idx),
                pin.ReferenceFrame.WORLD
            )

    def links_JInv(self):
        JInv_list = []

        for link_id in self.links_ids:
            Ji = link_worldJInv(link_id)
   
            JInv_list.append(Ji)

        return JInv_list

    def link_worldJInv(self,name_idx):

        J=self.link_worldJ

        return np.matmul(J.T,
                    np.linalg.inv(
                        np.matmul(J,J.T)
                        +self.DAMP*np.eye(6))
                )

    def eef_worldJ(self):

        return pin.getFrameJacobian(
                self.model,
                self.data,
                self.eef_id,
                pin.ReferenceFrame.WORLD
            )

    def eef_worldPos(self):

        return self.data.oMf[self.eef_id].translation


# if __name__ == '__main__':
#     base_dir = os.path.abspath(os.path.dirname(__file__) + '../../..')
#     robot_dir = os.path.join(base_dir, 'robots/darias_description/robots')
#     urdf_filename = os.path.join(robot_dir, 'darias_clean.urdf')
#
#     link_names = ['R_1_link', 'R_2_link', 'R_3_link']
#
#     robot = Robot(urdf_file=urdf_filename, link_name_list=link_names)
#     q = np.random.rand(7)
#     robot.update_kindyn(q=q)
#
#     print(robot)


# if __name__ == '__main__':
#     base_dir = os.path.abspath(os.path.dirname(__file__) + '../../..')
#     robot_dir = os.path.join(base_dir, 'robots/darias_description/robots')
#     urdf_filename = os.path.join(robot_dir, 'darias_clean.urdf')

#     link_names = ['R_1_link', 'R_2_link', 'R_3_link']

#     robot = Robot(urdf_file=urdf_filename, link_name_list=link_names)
#     q = np.random.rand(7)
#     robot.update_kindyn(q=q)

#     a=robot.links_J()
#     b=robot.links_JInv()

#     print(robot)
