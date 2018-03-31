#include <iostream>
#include <string>
#include <sstream>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/pcl_visualizer.h>

int main (int argc, char** argv)
{
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr input_cloud (new pcl::PointCloud<pcl::PointXYZRGBA>);

    pcl::visualization::PCLVisualizer viewer("viewer");
    viewer.addPointCloud<pcl::PointXYZRGBA> (input_cloud, "Output");
    std::string output_name =argv[1];

    while (!viewer.wasStopped())
    {
        try
        {
            if (pcl::io::loadPCDFile<pcl::PointXYZRGBA> ("//home/suhailps/bax_ws/src/Synthetic-dataset-generation-for-machine-learning-models/Pointclouds/merge_output/"+output_name+".pcd", *input_cloud) == -1)
                continue;

            viewer.spinOnce(500);
            viewer.updatePointCloud( input_cloud, "Output" );
         }
         catch (...)
         { }
    }

    return 0;
}


