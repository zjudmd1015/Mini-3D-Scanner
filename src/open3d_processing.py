#!/usr/bin/env python

import rospy
import numpy as np
import copy
from py3d import *

from std_msgs.msg import Int64
from geometry_msgs.msg import Vector3

import sys
import os
current_path = os.path.dirname(__file__)
module_path = os.path.join(current_path, '../lib')
sys.path.append(module_path)
from modern_robotics import MatrixLog3, so3ToVec, AxisAng3


class registerMulti:
    def __init__(self):
        self.cloud_index = 0
        self.initFlag = True
        self.goodResultFlag = True
        self.registrationCount = 0
        self.posWorldTrans = np.identity(4)
        self.posLocalTrans = np.identity(4)
# test for loop detection info
        self.detectTransLoop = np.identity(4)
# queue_size should be a little bit big, cause processing speed is not quick enough
        self.sub = rospy.Subscriber("pcd_save_done", Int64, self.callback, queue_size = 10)

        self.rotation_dir = Vector3()
        self.rotation_dir_init_flag = False
        self.sub_rotation_dir = rospy.Subscriber("plane_normal", Vector3, self.callback_dir, queue_size = 10)

    def callback(self, num):
        self.cloud_index = num.data
        self.registering()

    def callback_dir(self, temp_dir):
        self.rotation_dir = temp_dir
        self.rotation_dir_init_flag = True

    def registering(self):
        # print(self.initFlag)
        if self.initFlag == False:  # not the first cloud
            # read the new cloud
            print("----------------")
            print("Have registered {} clouds;".format(self.registrationCount))
            print("Processing cloud {}...".format(self.cloud_index))

            self.cloud2 = read_point_cloud("/home/dylan2/catkin_ws/src/scanner/data/{}.pcd".format(self.cloud_index))
            # self.cloud1 = read_point_cloud("/home/dylan2/catkin_ws/src/temp/pointCloudInRviz/data/{}.pcd".format(self.cloud_index-1))

            # get local transformation between two lastest clouds
            # source_temp = copy.deepcopy(self.cloud2)
            # target_temp = copy.deepcopy(self.cloud1)
            # print("cloud1: ",id(self.cloud1))
            self.posLocalTrans = self.registerLocalCloud(self.cloud1, self.cloud2)
            # self.posLocalTrans = self.registerLocalCloud(self.cloud1, self.cloud2)

            # if result is not good, drop it
            if self.goodResultFlag == True:
                # self.posLocalTrans = tempTrans
# test for loop detection info
                self.detectTransLoop = np.dot(self.posLocalTrans,self.detectTransLoop)
                # print ("==== loop detect trans ====")
                # print(self.detectTransLoop)
                # print ("==== ==== ==== ==== ==== ====")
                self.posWorldTrans =  np.dot(self.posWorldTrans, self.posLocalTrans)
                # update latest cloud
                self.cloud1 = copy.deepcopy(self.cloud2)
                self.cloud2.transform(self.posWorldTrans)
                self.cloud_base = self.cloud_base + self.cloud2

                # downsampling
                self.cloud_base = voxel_down_sample(self.cloud_base ,0.001)

                self.registrationCount += 1
                # save PCD file to local
                write_point_cloud("/home/dylan2/catkin_ws/src/scanner/data/result/registerResult.pcd", self.cloud_base ,write_ascii = False)

            else:
                pass

        # the first cloud
        else:
            self.cloud_base = read_point_cloud("/home/dylan2/catkin_ws/src/scanner/data/{}.pcd".format(self.cloud_index))
            self.cloud1 = copy.deepcopy(self.cloud_base)
            write_point_cloud("/home/dylan2/catkin_ws/src/scanner/data/result/registerResult.pcd", self.cloud_base ,write_ascii = False)
            if (self.rotation_dir_init_flag == True):
                self.initFlag = False

    def registerLocalCloud(self, target, source):
        '''
        function: get local transformation matrix
        input: two clouds
        output: transformation from target cloud to source cloud
        '''
        # print("target: ",id(target))
        source_temp = copy.deepcopy(source)
        target_temp = copy.deepcopy(target)
        # print("target_temp: ",id(target_temp))
        # source_temp = source
        # target_temp = target
        source_temp = voxel_down_sample(source_temp ,0.004)
        target_temp = voxel_down_sample(target_temp ,0.004)

        estimate_normals(source_temp, search_param = KDTreeSearchParamHybrid(
            radius = 0.1, max_nn = 30))
        estimate_normals(target_temp, search_param = KDTreeSearchParamHybrid(
            radius = 0.1, max_nn = 30))

        current_transformation = np.identity(4);
        # use Point-to-plane ICP registeration to obtain initial pose guess
        result_icp_p2l = registration_icp(source_temp, target_temp, 0.02,
                current_transformation, TransformationEstimationPointToPlane())
        # 0.1 is searching distance

        #'''TEST
        # current_transformation = result_icp.transformation
        # result_icp = registration_icp(source, target, 0.1,
        #     current_transformation, TransformationEstimationPointToPlane())
        #'''

        print("----------------")
        print("initial guess from Point-to-plane ICP registeration")
        print(result_icp_p2l)
        print(result_icp_p2l.transformation)

        p2l_init_trans_guess = result_icp_p2l.transformation
        print("----------------")
        # print("Colored point cloud registration")

################
#### testing registration
################
        # voxel_radius = [ 0.004,0.001 ];
        # max_iter = [ 50, 30 ];
        # current_transformation = p2l_init_trans_guess
        # for scale in range(2):
        #     iter = max_iter[scale]
        #     radius = voxel_radius[scale]
        #     source_down = voxel_down_sample(source_temp, radius)
        #     target_down = voxel_down_sample(target_temp, radius)
        #     estimate_normals(source_down, KDTreeSearchParamHybrid(
        #         radius = radius * 2, max_nn = 30))
        #     estimate_normals(target_down, KDTreeSearchParamHybrid(
        #         radius = radius * 2, max_nn = 30))
        #     result_icp = registration_colored_icp(source_down, target_down,
        #         radius, current_transformation,
        #         ICPConvergenceCriteria(relative_fitness = 1e-6,
        #         relative_rmse = 1e-6, max_iteration = iter))
        #     current_transformation = result_icp.transformation

#################
#### original one
#################
        # result_icp = registration_colored_icp(source_temp, target_temp,
        #     0.001, p2l_init_trans_guess,
        #     ICPConvergenceCriteria(relative_fitness = 1e-6,
        #     relative_rmse = 1e-6, max_iteration = 50))
#################
#################
#################

#################
#### double nomral ICP
#################
        result_icp = registration_icp(source_temp, target_temp, 0.01,
                p2l_init_trans_guess, TransformationEstimationPointToPlane())
#         result_icp = registration_icp(source_temp, target_temp, 0.002,
#                 p2l_init_trans_guess, TransformationEstimationPointToPlane(),ICPConvergenceCriteria(relative_fitness = 1e-6,relative_rmse = 1e-6, max_iteration = 50))
# #################
#################
#################


        # result_icp = registration_colored_icp(source, target,
        #     0.02, p2l_init_trans_guess)
        print(result_icp)
        print(result_icp.transformation)
        # print(result_icp.fitness)

        # draw_registration_result_original_color(
        #         source, target, result_icp.transformation)


        #### write intermediate result for test,
        #### but found it will make registration result worse...
        #### ---->>>> cause source.transform will change the source!!!!!!
        # source.transform(result_icp.transformation)
        # temp_cloud = target + source
        # write_point_cloud("/home/dylan2/catkin_ws/src/temp/pointCloudInRviz/data/result/{}-{}.pcd".format(self.cloud_index,self.cloud_index-1), temp_cloud , write_ascii = False)


        # print("*****")
        # print(source)
        # print(target)

        # print(result_icp.correspondence_set)
######################################
#####   original kick-out rule
######################################
        # self.goodResultFlag = True
        # if (result_icp.fitness < 0.9 * result_icp_p2l.fitness):
        #     return p2l_init_trans_guess           # when result of colored ICP is bad, use p2l ICP
        # else:
        #     return result_icp.transformation

######################################
#####   kick-out rule
######################################
#### New rule:
#### 1/ rotation is out of plane (5 degree, or 0.087266 in radians);
#### 2/ too big rotation;
#### 3/ too big translation;

# first calculate what is the rotation direction and rotation angle
        tf = result_icp.transformation
        R = tf[:3,:3]  # rotation matrix
        so3mat = MatrixLog3(R)
        omg = so3ToVec(so3mat)
        R_dir, theta = AxisAng3(omg) # rotation direction
                # rotation angle (in radians)
        theta_degree = theta / np.pi * 180 # in degree
        angle_with_pl_norm = self.cal_angle(self.rotation_dir, R_dir)

        trans_tol= 0.5  # transformation tolerance
        rotation_tol = 30 # 30 degrees
        # angle_with_pl_norm_tol = 0.087266 # in radians (= 5 degrees)
        # angle_with_pl_norm_tol = 0.174533 # in radians (= 10 degrees)
        angle_with_pl_norm_tol = 0.35 # in radians (= 20 degrees)
        if ( tf[0,3] > trans_tol or tf[0,3] < -trans_tol or \
             tf[1,3] > trans_tol or tf[1,3] < -trans_tol or \
             tf[2,3] > trans_tol or tf[2,3] < -trans_tol ):
            self.goodResultFlag = False
            print(">>>>>>>>>>>>>>>>>>>>>>>>>>>>>>")
            # print("here in 1 ")
            rospy.logwarn('Something wrong with 1/ translation : (, turn back a little bit...')
            rospy.logwarn('>> the translation is [{},{},{}]'.format(tf[0,3],tf[1,3],tf[2,3]))
            return np.identity(4)
        elif ( theta_degree > rotation_tol or \
               theta_degree < - rotation_tol):
            self.goodResultFlag = False
            print(">>>>>>>>>>>>>>>>>>>>>>>>>>>>>>")
            # print("here in 2 ")
            rospy.logwarn('Something wrong with 2/ rotation angle : (, turn back a little bit...')
            rospy.logwarn('>> the rotation angle is {} (in degrees)'.format(theta_degree))
            return np.identity(4)
        elif ( angle_with_pl_norm > angle_with_pl_norm_tol):
            self.goodResultFlag = False
            print(">>>>>>>>>>>>>>>>>>>>>>>>>>>>>>")
            print("here in 3 ")
            print(" angle with pl norm")
            print(angle_with_pl_norm)
            rospy.logwarn('Something wrong with 3/ rotation axis : (, turn back a little bit...')
            rospy.logwarn('>> the rotation axis is {} (in radians) with plane normal'.format(angle_with_pl_norm))
            return np.identity(4)
        else:
            self.goodResultFlag = True
            print(">>>>>>>>>>>>>>>>>>>>>>>>>>>>>>")
            # print("here in 4 ")
            return result_icp.transformation

        # print(">>>>>>>>>>>>>>>>>>>>>>>>>>>>>>")
        # print(self.goodResultFlag)

        # if result_icp.fitness > -1:
        #     self.goodResultFlag = True
        #     return result_icp.transformation
        # else:
        #     self.goodResultFlag = False
        #     return np.identity(4)

    def cal_angle(self,pl_norm, R_dir):
        angle_in_radians = \
            np.arccos(
                np.abs(pl_norm.x*R_dir[0]+ pl_norm.y*R_dir[1] + pl_norm.z*R_dir[2])
                )

        return angle_in_radians


def main():
    registerMulti()
    rospy.init_node('open3d_processing')
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down ROS open3d_processing")

if __name__ == "__main__":
    main()
