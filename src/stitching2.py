import sys
#path to the directory containing py3d.so
sys.path.append("/home/suhailps/Code/Open3D/build/lib")
sys.path.append("../..")
from py3d import *
import numpy as np
import copy
import os



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
        self.fullfull()



    def fullpair(self):
        # self.cloud_index = num.data
        initial_trans=np.identity(4)
        final = read_point_cloud("../Pointclouds/scan_output/cloud1.pcd")
        target = final

        for i in xrange(1,500):
            if True:
                src_name="../Pointclouds/scan_output/cloud"+str(i*3)+".pcd"
                source = read_point_cloud(src_name)
                print("Registering cloud"+str(i*3)+".pcd")

                target = voxel_down_sample(target, 0.001)


                self.posLocalTrans = self.registerLocalCloud(target, source)

                if self.goodResultFlag == True:

                    self.detectTransLoop = np.dot(self.posLocalTrans,self.detectTransLoop)

                    self.posWorldTrans =  np.dot(self.posWorldTrans, self.posLocalTrans)

                    target = copy.deepcopy(source)
                    source.transform(self.posWorldTrans)
                    final = final + source

                    final = voxel_down_sample(final ,0.001)

                    self.registrationCount += 1

                    write_point_cloud("../Pointclouds/merge_output/merged.pcd",final)
                    write_point_cloud("../Pointclouds/merge_output/merged.ply",final)

                else:
                    pass


            else:

                self.initFlag = False


    def fullfull(self):
        # self.cloud_index = num.data
        initial_trans=np.identity(4)
        final = read_point_cloud("../Pointclouds/scan_output/cloud1.pcd")
        target = final

        for i in xrange(1,500):
            if True:
                src_name="../Pointclouds/scan_output/cloud"+str(i*3)+".pcd"
                source = read_point_cloud(src_name)
                print("Registering cloud"+str(i*3)+".pcd")

                target = voxel_down_sample(target, 0.001)


                self.posLocalTrans = self.registerLocalCloud(target, source)

                if self.goodResultFlag == True:

                    self.detectTransLoop = np.dot(self.posLocalTrans,self.detectTransLoop)

                    self.posWorldTrans =  np.dot(self.posWorldTrans, self.posLocalTrans)

                    # target = copy.deepcopy(source)
                    source.transform(self.posLocalTrans)
                    target = target + source

                    target = voxel_down_sample(target ,0.001)

                    self.registrationCount += 1

                    write_point_cloud("../Pointclouds/merge_output/merged.pcd",target)
                    write_point_cloud("../Pointclouds/merge_output/merged.ply",target)

                else:
                    pass


            else:

                self.initFlag = False

    def registerLocalCloud(self, target, source):

        source_temp = copy.deepcopy(source)
        target_temp = copy.deepcopy(target)


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

        # print("----------------")
        # print("initial guess from Point-to-plane ICP registeration")
        # print(result_icp_p2l)
        # print(result_icp_p2l.transformation)

        p2l_init_trans_guess = result_icp_p2l.transformation
        print("----------------")
        print("Colored point cloud registration")

#################
        result_icp = registration_colored_icp(source_temp, target_temp,
            0.001, p2l_init_trans_guess,
            ICPConvergenceCriteria(relative_fitness = 1e-6,
            relative_rmse = 1e-6, max_iteration = 500))
        # result_icp = registration_icp(source_temp, target_temp, 0.009,
        #         p2l_init_trans_guess, TransformationEstimationPointToPoint())

        # print(result_icp)
        print(result_icp.fitness)

        print(">>>>>>>>>>>>>>>>>>>>>>>>>>>>>>")
        print(self.goodResultFlag)
        if result_icp.fitness > -1:
            self.goodResultFlag = True
            return result_icp.transformation
        else:
            self.goodResultFlag = False
            return np.identity(4)



    def registerLocalCloud2(self, target, source):

        source_temp = copy.deepcopy(source)
        target_temp = copy.deepcopy(target)


        estimate_normals(source_temp, search_param = KDTreeSearchParamHybrid(
            radius = 0.1, max_nn = 30))
        estimate_normals(target_temp, search_param = KDTreeSearchParamHybrid(
            radius = 0.1, max_nn = 30))

        print "Computing fpfh"

        source_fpfh = compute_fpfh_feature(source,
                KDTreeSearchParamHybrid(radius = 0.25, max_nn = 100))
        target_fpfh = compute_fpfh_feature(target,KDTreeSearchParamHybrid(radius = 0.25, max_nn = 100))

        result_ransac = registration_ransac_based_on_feature_matching(source, target, source_fpfh, target_fpfh, 0.075,TransformationEstimationPointToPoint(False), 4,[CorrespondenceCheckerBasedOnEdgeLength(0.9),CorrespondenceCheckerBasedOnDistance(0.03)],RANSACConvergenceCriteria(400000, 500))

        print("----------------")
        print("initial guess from Point-to-plane ICP registeration")
        print(result_ransac)


        p2l_init_trans_guess = result_ransac.transformation
        print("----------------")
        print("Colored point cloud registration")

#################
        result_icp = registration_colored_icp(source_temp, target_temp,
            0.001, p2l_init_trans_guess,
            ICPConvergenceCriteria(relative_fitness = 1e-6,
            relative_rmse = 1e-6, max_iteration = 50))

        print(result_icp)
        print(result_icp.transformation)

        print(">>>>>>>>>>>>>>>>>>>>>>>>>>>>>>")
        print(self.goodResultFlag)
        if result_icp.fitness > -1:
            self.goodResultFlag = True
            return result_icp.transformation
        else:
            self.goodResultFlag = False
            return np.identity(4)


if __name__ == "__main__":
    numArgs = len(sys.argv)
    if numArgs == 2:
        filePath = sys.argv[1]
        if not os.path.exists(filePath):
            print 'Filepath does not exist'
        elif not os.path.isdir(filePath):
            print 'Filepath is not a directory!'
        else:
            sys.path.append(filePath)
            from py3d import *
    else:
        print 'command format = python stitching.py <path to directory containing py3d.so file>'

    registerMulti()






