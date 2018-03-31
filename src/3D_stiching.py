import sys
import numpy as np
import copy
import os
import argparse
import os.path





class stitching:


    def __init__(self,argument):

        self.withind_tolerance = True
        self.global_transformation = np.identity(4)
        self.local_transformation = np.identity(4)
        self.global_algorithm='ransac'#'ransac'#'icp'
        self.local_algorithm='icp'#icp
        self.merge_type='pairwise'
        self.output="output"
        self.tolerance=0.0
        self.count=0


        if argument.local_algorithm:
            print("You have used {0} for local registration".format(argument.local_algorithm))

            self.local_algorithm=argument.local_algorithm
            status = True
        if argument.global_algorithm:
            print("You have used {0} for Global registration".format(argument.global_algorithm))
            self.global_algorithm=argument.global_algorithm
            status = True

        if argument.output:
            self.output=argument.output
            print("Output file name is {0}".format(argument.output))
        if argument.tolerance:
            self.tolerance=argument.tolerance
            print("Tolerance threshold is {0}".format(self.tolerance))


        if argument.merge_type:
            print("You have used {0} as merge type".format(argument.merge_type))
            if argument.merge_type=='pairwise':
                self.stitching_pairwise()
            else:
                self.stitching_full()
            status = True
        if not status:
            print("Maybe you want to use some arguments ?")


    def stitching_pairwise(self):
        final = read_point_cloud("../Pointclouds/scan_output/cloud1.pcd")
        target = final

        for i in xrange(1,60):#looping all pointclouds
            if True:
                #getting source cloud
                src_name="../Pointclouds/scan_output/cloud"+str(i*3)+".pcd"
                while True:
                    if os.path.isfile(src_name) :
                        source = read_point_cloud(src_name)
                        break
                    else:
                        print 'Waiting for cloud'+str(i*3)+".pcd"

                print("Registering cloud"+str(i*3)+".pcd")

                # target = voxel_down_sample(target, 0.001)


                self.local_transformation = self.register_modular(target, source)

                if self.withind_tolerance == True:

                    #accumulating world transforating by merging each local transform
                    self.global_transformation =  np.dot(self.global_transformation, self.local_transformation)

                    #making  source the next target forpairwise stiching
                    target = copy.deepcopy(source)
                    source.transform(self.global_transformation)
                    final = final + source

                    #keeping final light by downsamplig
                    final = voxel_down_sample(final ,0.001)

                    write_point_cloud("../Pointclouds/merge_output/"+self.output+".pcd",final)
                    write_point_cloud("../Pointclouds/merge_output/"+self.output+".ply",final)



    def stitching_full(self):
        final = read_point_cloud("../Pointclouds/scan_output/cloud1.pcd")
        target = final

        for i in xrange(1,160):#looping all pointclouds
            if True:
                #getting source cloud
                src_name="../Pointclouds/scan_output/cloud"+str(i*3)+".pcd"
                source = read_point_cloud(src_name)
                print("Registering cloud"+str(i*3)+".pcd")

                self.count=0
                while True:
                    self.local_transformation = self.register_modular(target, source)
                    if self.withind_tolerance == True:

                        #making  source the next target forpairwise stiching
                        source.transform(self.local_transformation)
                        final = final + source

                        #keeping final light by downsamplig
                        # final = voxel_down_sample(final ,0.001)
                        #making fianl as the target
                        target = copy.deepcopy(final)
                        write_point_cloud("../Pointclouds/merge_output/"+self.output+".pcd",final)
                        write_point_cloud("../Pointclouds/merge_output/"+self.output+".ply",final)
                        break
                    else:
                        self.count=self.count+1
                        if self.global_algorithm=='icp':
                            print ("Skipping  cloud"+str(i*3)+".pcd")
                            break
                        elif self.count>10:
                            print ("Skipping  cloud"+str(i*3)+".pcd")
                            break
                        continue




    def register_modular(self, target, source):

        source_temp = copy.deepcopy(source)
        target_temp = copy.deepcopy(target)

        #estimating normals
        estimate_normals(source_temp, search_param = KDTreeSearchParamHybrid(
            radius = 0.1, max_nn = 30))
        estimate_normals(target_temp, search_param = KDTreeSearchParamHybrid(
            radius = 0.1, max_nn = 30))

        #Global registration using coarse ICP
        global_result = self.global_registration(target_temp, source_temp)


        print 'Global registration fitness =' ,global_result.fitness

        #local registration using colored ICP
        local_result = self.local_registration(target_temp, source_temp,global_result.transformation)

        print 'Local registration fitness =',local_result.fitness

        if local_result.fitness >float(self.tolerance):
            self.withind_tolerance = True
            return local_result.transformation
        else:
            self.withind_tolerance = False
            return None

    def global_registration(self, target, source):

        #identity matrix as initial transformation
        current_transformation = np.identity(4);
        if self.global_algorithm=='icp':
            print 'Global registration with coarse ICP'
            #Global registration using coarse ICP
            global_result = registration_icp(source, target, 0.01,
                    current_transformation, TransformationEstimationPointToPlane())
            return global_result

        if self.global_algorithm=='ransac':
            print 'Global registration with RANSAC'


            source_down = voxel_down_sample(source, 0.002)
            target_down = voxel_down_sample(target, 0.002)

            if self.count==0:
                print "Computing fpfh"
                self.source_fpfh = compute_fpfh_feature(source_down,
                        KDTreeSearchParamHybrid(radius = 0.05, max_nn = 100))
                self.target_fpfh = compute_fpfh_feature(target_down,
                        KDTreeSearchParamHybrid(radius = 0.05, max_nn = 100))

            #Global registration using coarse ICP
            result_ransac = registration_ransac_based_on_feature_matching(
                    source_down, target_down, self.source_fpfh, self.target_fpfh, 0.075,
                    TransformationEstimationPointToPoint(False), 4,
                    [CorrespondenceCheckerBasedOnDistance(0.01)],
                    RANSACConvergenceCriteria(400000, 500))
            return result_ransac



    def local_registration(self, target, source,initial_transformation):
        if self.local_algorithm=='icp_color':
            #local registration using colored ICP
            print 'Local registration with colored ICP'
            local_result = registration_colored_icp(source, target,
                0.001, initial_transformation,
                ICPConvergenceCriteria(relative_fitness = 1e-6,
                relative_rmse = 1e-6, max_iteration = 500))

            return local_result

        elif self.local_algorithm=='icp':
            #local registration using colored ICP
            print 'Local registration with fine ICP'
            local_result = registration_icp(source, target, 0.01,
                    initial_transformation, TransformationEstimationPointToPlane())

            return local_result

if __name__ == "__main__":

    parser = argparse.ArgumentParser(description = "Description for my parser")
    parser.add_argument("-la", "--local_algorithm", help = "Example: Local_algorithm", required = False, default = "icp_color")
    parser.add_argument("-ga", "--global_algorithm", help = "Example: Global algorithm", required = False, default = "icp")
    parser.add_argument("-m", "--merge_type", help = "Example: Merge type", required = False, default = "pairwise")
    parser.add_argument("-p", "--path", help = "Example: Path to py3D.so", required = False, default = "/home/suhailps/Code/Open3D/build/lib")
    parser.add_argument("-o", "--output", help = "Example: Output file name", required = False, default = "output")
    parser.add_argument("-t", "--tolerance", help = "Example:Tolerance threshold", required = False, default = "0.0")


    argument = parser.parse_args()
    status = False

    if argument.path:
        print("You have used {0} for path to py3D.so".format(argument.path))
        sys.path.append(argument.path)
        from py3d import *



    stitching(argument)

