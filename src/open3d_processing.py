#!/usr/bin/env python

import rospy
import numpy as np
import copy
from py3d import *

from std_msgs.msg import Int64


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

    def callback(self, num):
        self.cloud_index = num.data
        self.registering()

    def registering(self):
        # print(self.initFlag)
        if self.initFlag == False:  # not the first cloud
            # read the new cloud
            print("----------------")
            print("Have registered {} clouds;".format(self.registrationCount))
            print("Processing cloud {}...".format(self.cloud_index))

            self.cloud2 = read_point_cloud("/home/dylan2/catkin_ws/src/temp/pointCloudInRviz/data/{}.pcd".format(self.cloud_index))
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
                write_point_cloud("/home/dylan2/catkin_ws/src/temp/pointCloudInRviz/data/result/registerResult.pcd", self.cloud_base ,write_ascii = False)

            else:
                pass

        # the first cloud
        else:
            self.cloud_base = read_point_cloud("/home/dylan2/catkin_ws/src/temp/pointCloudInRviz/data/{}.pcd".format(self.cloud_index))
            self.cloud1 = copy.deepcopy(self.cloud_base)

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
        print("Colored point cloud registration")

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
##### original one
#################
        result_icp = registration_colored_icp(source_temp, target_temp,
            0.001, p2l_init_trans_guess,
            ICPConvergenceCriteria(relative_fitness = 1e-6,
            relative_rmse = 1e-6, max_iteration = 50))
#################
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
        print(">>>>>>>>>>>>>>>>>>>>>>>>>>>>>>")
        print(self.goodResultFlag)
        if result_icp.fitness > -1:
            self.goodResultFlag = True
            return result_icp.transformation
        else:
            self.goodResultFlag = False
            return np.identity(4)


def main():
    registerMulti()
    rospy.init_node('open3d_processing')
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down ROS open3d_processing")

if __name__ == "__main__":
    main()
