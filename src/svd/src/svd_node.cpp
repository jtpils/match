#include <ros/ros.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/registration/transformation_estimation_svd.h>
#include <iostream>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "svd");
    ros::NodeHandle nh;

    pcl::PointCloud<pcl::PointXYZI> laser1;
    pcl::PointCloud<pcl::PointXYZI> laser2;
    pcl::registration::TransformationEstimationSVD<pcl::PointXYZI, pcl::PointXYZI, float> svd;
    
    Eigen::Matrix4f guess;
    svd.estimateRigidTransformation(laser1, laser2, guess);

    std::cout << guess << std::endl;
    return 0;
}

