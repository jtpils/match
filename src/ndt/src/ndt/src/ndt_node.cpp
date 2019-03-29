#include <iostream>
#include <ros/ros.h>
#include <pcl/point_types.h>
#include <pcl/common/transforms.h>
#include <pcl/registration/ndt.h>
#include <pcl/sample_consensus/sac_model_line.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/common/pca.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>

int main(int argc, char **argv)
{
    pcl::PointCloud<pcl::PointXYZI> laser1;
    pcl::PointCloud<pcl::PointXYZI> laser2;

    pcl::io::loadPCDFile("200.pcd", laser1);
    pcl::io::loadPCDFile("202.pcd", laser2);
    
    pcl::VoxelGrid<pcl::PointXYZI> filters;
    filters.setInputCloud(laser1.makeShared());
    filters.setLeafSize(0.1, 0.1, 0.1);
    filters.filter(laser1);

    std::cout << "ndt start ..." << std::endl;
    pcl::NormalDistributionsTransform<pcl::PointXYZI, pcl::PointXYZI> ndt;
    ndt.setInputTarget(laser2.makeShared());
    ndt.setTransformationEpsilon(0.0001);
    ndt.setStepSize(0.1);
    ndt.setResolution(1.0);
    ndt.setMaximumIterations(100);
    ndt.setInputSource(laser1.makeShared());
    static Eigen::Matrix4f guess = Eigen::Matrix4f::Identity();

    //guess(1,3) = -2.5;
    ndt.align(laser1, guess);

    std::cout << ndt.getFinalTransformation() << std::endl;
    std::cout << "ndt end ..." << std::endl;
    

    pcl::io::savePCDFile("final.pcd", laser1 + laser2);
    return 0;
}

