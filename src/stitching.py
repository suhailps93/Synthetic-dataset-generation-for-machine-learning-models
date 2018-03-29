import sys
#path to the directory containing py3d.so
#sys.path.append("/home/suhailps/Code/Open3D/build/lib")
# sys.path.append("../..")
# from py3d import *
import numpy as np
import copy
import os
# global_trans = np.asarray([[1.0, 0.0, 0.0, 0.0],
# 							[0.0, 1.0, 0.0, 0.0],
# 							[0.0, 0.0, 1.0, 0.0],
# 							[0.0, 0.0, 0.0, 1.0]])
def trans(source,  transformation):

	source.transform(transformation)

	return source

def full():
	final = read_point_cloud("../Pointclouds/scan_output/1.pcd")
	target = final


	for i in xrange(1,500):
		registration=0
		print("i=",i)

		src_name="../Pointclouds/scan_output/"+str(i+1)+".pcd"
		source = read_point_cloud(src_name)
		print(src_name)
		# print(tgt_name)

		source_down = source
		target_down = target

		if i>50:
			a=i*1.0/10000
			target_down = voxel_down_sample(target,a)


		# source_down = voxel_down_sample(source, 0.0005)
		# target_down = voxel_down_sample(target, 0.001)

		print "Estimating Normals"

		estimate_normals(source_down, KDTreeSearchParamHybrid(
				radius = 0.1, max_nn = 30))
		estimate_normals(target_down, KDTreeSearchParamHybrid(
				radius = 0.1, max_nn = 30))

		print "Computing fpfh"

		source_fpfh = compute_fpfh_feature(source_down,
				KDTreeSearchParamHybrid(radius = 0.25, max_nn = 100))
		target_fpfh = compute_fpfh_feature(target_down,
				KDTreeSearchParamHybrid(radius = 0.25, max_nn = 100))
		count=1
		tolerance=0.80
		r=4000
		while True:
			print "Ransac registration"
			result_ransac = registration_ransac_based_on_feature_matching(
					source_down, target_down, source_fpfh, target_fpfh, 0.075,
					TransformationEstimationPointToPoint(False), 4,
					[CorrespondenceCheckerBasedOnEdgeLength(0.9),
					CorrespondenceCheckerBasedOnDistance(0.03)],
					RANSACConvergenceCriteria(r, 500))
			print "Ransac merge fitness=",result_ransac.fitness
			if result_ransac.fitness<0.96 and r<4000000:
				r=r*10
				continue


			print "ICP Registration"
			result_icp = registration_icp(source, target, 0.01,
					result_ransac.transformation,
					TransformationEstimationPointToPlane())

			print"ICP Fitness=",result_icp.fitness

			if (result_icp.fitness>tolerance):
				count=1
				tolerance=0.80

				target=target+trans(source,  result_icp.transformation)

				print("full_0004.pcd tgt down 0.0005")
				write_point_cloud("../Pointclouds/merge_output/merged.pcd",target)
				write_point_cloud("../Pointclouds/merge_output/merged.ply",target)

				break


			else:
				count=count+1
				if count%5==0:
					tolerance=tolerance-0.01
					print"tolerance reduced to================== ", tolerance

                    # if true:
                    #     continue
                    # if tolerance<.95:
                    #     # print"skipping pcd====================== "
                    #     continue


def full2():
	initial_trans=np.identity(4)
	final = read_point_cloud("../Pointclouds/scan_output/cloud1.pcd")
	target = final


	for i in xrange(1,500):
		registration=0

		src_name="../Pointclouds/scan_output/cloud"+str(i*3)+".pcd"
		source = read_point_cloud(src_name)
		print("Registering cloud"+str(i*3)+".pcd")

		target = voxel_down_sample(target, 0.001)
		print "Estimating Normals"


		estimate_normals(source, search_param = KDTreeSearchParamHybrid(
			radius = 0.1, max_nn = 30))
		estimate_normals(target, search_param = KDTreeSearchParamHybrid(
			radius = 0.1, max_nn = 30))


		print "ICP coarse"

		result_icp = registration_colored_icp(source, target,
			0.001, initial_trans,
			ICPConvergenceCriteria(relative_fitness = 1e-6,
			relative_rmse = 1e-6, max_iteration = 50))

		print"ICP  Fitness=",result_icp.fitness
		print "Colored ICP coarse"
		initial_trans=result_icp.transformation
		result_icp_color = registration_colored_icp(source, target,
			0.001, initial_trans,
			ICPConvergenceCriteria(relative_fitness = 1e-6,
			relative_rmse = 1e-6, max_iteration = 50))

		print"ICP color Fitness=",result_icp_color.fitness
		tolerance=0.98
		if True:#(result_icp.fitness>tolerance):
			tolerance=0.80

			target=target+trans(source,  result_icp.transformation)
			write_point_cloud("../Pointclouds/merge_output/merged.pcd",target)
			write_point_cloud("../Pointclouds/merge_output/merged.ply",target)







def pairwise():

	final = read_point_cloud("../Pointclouds/scan_output/1.pcd")
	a=[]

	for i in xrange(100,500):
		registration=0
		print("i=",i)




		src_name="../Pointclouds/scan_output/"+str(i+1)+".pcd"
		tgt_name="../Pointclouds/scan_output/"+str(i)+".pcd"

		source = read_point_cloud(src_name)
		target = read_point_cloud(tgt_name)

		print(src_name)
		print(tgt_name)

		source_down = source
		target_down = target

		# source_down = voxel_down_sample(source, 0.002)
		# target_down = voxel_down_sample(target, 0.002)

		print "Estimating Normals"
		estimate_normals(source_down, KDTreeSearchParamHybrid(
				radius = 0.02, max_nn = 30))
		estimate_normals(target_down, KDTreeSearchParamHybrid(
				radius = 0.02, max_nn = 30))

		print "Computing fpfh"
		source_fpfh = compute_fpfh_feature(source_down,
				KDTreeSearchParamHybrid(radius = 0.25, max_nn = 100))
		target_fpfh = compute_fpfh_feature(target_down,
				KDTreeSearchParamHybrid(radius = 0.25, max_nn = 100))
		r=400000
		count=1
		tolerance=0.99
		while True:
			print "Ransac registration"
			result_ransac = registration_ransac_based_on_feature_matching(
					source_down, target_down, source_fpfh, target_fpfh, 0.075,
					TransformationEstimationPointToPoint(False), 4,
					[CorrespondenceCheckerBasedOnDistance(0.01)],
					RANSACConvergenceCriteria(r, 500))
            # result_ransac = registration_ransac_based_on_feature_matching(
            #         source_down, target_down, source_fpfh, target_fpfh, 0.075,
            #         TransformationEstimationPointToPoint(False), 4,
            #         [CorrespondenceCheckerBasedOnEdgeLength(0.9),
            #         CorrespondenceCheckerBasedOnDistance(0.075)],
            #         RANSACConvergenceCriteria(r, 500))
			if result_ransac.fitness<0.99 and r<4000000:
				r=r*10
				continue

			print "Ransac merge fitness=",result_ransac.fitness
			# draw_registration_result(source, target, result_ransac.transformation)

			print "ICP Registration"
			result_icp = registration_icp(source, target, 0.01,
					result_ransac.transformation,
					TransformationEstimationPointToPlane())


		    # result_icp = registration_colored_icp(source, target,radius, current_transformation,ICPConvergenceCriteria(relative_fitness = 1e-6,relative_rmse = 1e-6, max_iteration = iter))
			# result_icp=register(i)
			print"ICP Fitness=",result_icp.fitness
			# if(result_icp.fitness<result_ransac.fitness):
			# 	result_icp=result_ransac
			# 	print("Reverted Fitness=%f",result_icp.fitness)
			if (result_icp.fitness>tolerance):
				count=1
				tolerance=0.99
				a.append(result_icp.transformation)
				print(len(a))
				print ("icp merge fitness=%f",result_icp.fitness)

				# if i>65:
				# draw_registration_result(source, target, result_icp.transformation)

				break


			else:
				count=count+1
				if count%5==0:
					tolerance=tolerance-0.01
					print"tolerance reduced to====================== ", tolerance
				continue

		print("Inner i=",i)
		p=read_point_cloud("../Pointclouds/scan_output/"+str(i+1)+".pcd")
		j=i-101
		while j>=0:

			p=trans(p,  a[j])
			j=j-1
		final=final+p
		print "tide__from_100.pcd pairwise "
		write_point_cloud("../Pointclouds/merge_output/merged.pcd",final)
		write_point_cloud("../Pointclouds/merge_output/merged.ply",final)

if __name__ == "__main__":
	numArgs = len(sys.argv)
	if numArgs == 2:
		# Retrieve the input directory
		filePath = sys.argv[1]

		# Check input file path exists and is a directory
		if not os.path.exists(filePath):
			print 'Filepath does not exist'
		elif not os.path.isdir(filePath):
			print 'Filepath is not a directory!'
		else:
		# Merge the shapefiles within the filePath
			sys.path.append(filePath)
			from py3d import *
	else:
		print 'command format = python stitching.py <path to directory containing py3d.so file>'

	full2()
	# pairwise()
