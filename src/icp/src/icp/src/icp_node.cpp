#include <ros/ros.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <pcl/io/pcd_io.h>
#include <fstream>
#include <string.h>
#include <iostream>

using namespace std;


int main(int argc, char **argv)
{
    ros::init(argc, argv, "icp");

    //pcl::PointCloud<pcl::PointXYZ> laser1 = readfile("09_29_200/plane.pcd");
    //pcl::PointCloud<pcl::PointXYZ> laser2 = readfile("09_29_202/plane.pcd");
    pcl::PointCloud<pcl::PointXYZ> laser1, laser2;
    pcl::io::loadPCDFile("200.pcd", laser1);
    pcl::io::loadPCDFile("202.pcd", laser2);


    pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;  //创建ICP的实例类
    icp.setInputSource(laser1.makeShared());
    icp.setInputTarget(laser2.makeShared());
    icp.setMaxCorrespondenceDistance(100);
    icp.setTransformationEpsilon(1e-10);
    //icp.setEuclideanFitnessEpsilon(0.001);
    icp.setMaximumIterations(100);
    icp.align(laser1);

    std::cout << icp.getFinalTransformation() << std::endl;
    return 0;
}

