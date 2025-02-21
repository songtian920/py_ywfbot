import numpy as np
import pandas as pd
import math
from scipy.spatial.transform import Rotation as R


class tool_cal():
    def __init__(self):
        """
		load data from csv
		tool_points(0~5) : use robot effectors to touch the same points in the world
                      and record the pos
        tool_poses_tran(0~3):count tanslation
        tool_poses_rot(3~5):count rotation
		"""
        with open("tool_data.csv") as file:
            tool_poses = pd.read_csv(file, header=None)
            tool_poses = np.array(tool_poses)

            # cal translation
            self.tran_tran = []
            self.tran_rotm = []
            tool_poses_tran = tool_poses[0:4, :]
            for pose in tool_poses_tran:
                # set translation
                self.tran_tran.append(np.array([[pose[0]], [pose[1]], [pose[2]]]))

                # set rotation
                r = R.from_euler('xyz', np.array([pose[3], pose[4], pose[5]]), degrees=True)
                self.tran_rotm.append(r.as_dcm())

            tool_tran = self.cal_tran()

            # cal rotation
            self.rot_tran = []
            self.rot_rotm = []
            tool_poses_rot = tool_poses[3:6, :]
            for pose in tool_poses_rot:
                # set translation
                self.rot_tran.append(np.array([[pose[0]], [pose[1]], [pose[2]]]))

                # set rotation
                r = R.from_euler('xyz', np.array([pose[3], pose[4], pose[5]]), degrees=True)
                self.rot_rotm.append(r.as_dcm())

            tool_rot = self.cal_rotm(tool_tran)

            # get transformation
            tool_T = np.array(np.zeros((4, 4)))
            tool_T[0:3, 0:3] = tool_rot
            tool_T[0:3, 3:] = tool_tran
            tool_T[3:, :] = [0, 0, 0, 1]
            #print tool_T

    def cal_tran(self):
        tran_data = []
        rotm_data = []
        for i in range(len(self.tran_tran) - 1):
            tran_data.append(self.tran_tran[i + 1] - self.tran_tran[i])
            rotm_data.append(self.tran_rotm[i] - self.tran_rotm[i + 1])

        L = np.array(np.zeros((3, 3)))
        R = np.array(np.zeros((3, 1)))
        for i in range(len(tran_data)):
            L = L + np.dot(rotm_data[i], rotm_data[i])
            R = R + np.dot(rotm_data[i], tran_data[i])

        return np.linalg.inv(L).dot(R)

    def cal_rotm(self, tran):
        # centre
        P_otcp_To_B = np.dot(self.rot_rotm[0], tran) + self.rot_tran[0]

        # cal the dircction vector of x
        P_xtcp_To_B = np.dot(self.rot_rotm[1], tran) + self.rot_tran[1]
        vector_X = P_xtcp_To_B - P_otcp_To_B
        dire_vec_x_o = np.linalg.inv(self.rot_rotm[0]).dot(vector_X) / np.linalg.norm(vector_X)

        # cal the dircction vector of z
        P_ztcp_To_B = np.dot(self.rot_rotm[2], tran) + self.rot_tran[2]
        vector_Z = P_ztcp_To_B - P_otcp_To_B
        dire_vec_z_o = np.linalg.inv(self.rot_rotm[0]).dot(vector_Z) / np.linalg.norm(vector_Z)

        # cal the dircction vector of y
        dire_vec_y_o = np.cross(dire_vec_z_o.T, dire_vec_x_o.T)

        # modify the dircction vector of z
        dire_vec_z_o = np.cross(dire_vec_x_o.T, dire_vec_y_o)

        # cal rotation matrix
        tool_rot = np.array(np.zeros((3, 3)))
        tool_rot[:, 0] = dire_vec_x_o.T
        tool_rot[:, 1] = dire_vec_y_o
        tool_rot[:, 2] = dire_vec_z_o
        return tool_rot


if __name__ == "__main__":
    tool_cal()