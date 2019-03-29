#include <ros/ros.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/registration/transformation_estimation_lm.h>
#include <iostream>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "lm");
    ros::NodeHandle nh;

    pcl::PointCloud<pcl::PointXYZ> laser1, laser2;
    for(int i = 0; i < 10; i ++)
    {
        pcl::PointXYZ pointOri, pointSel;
        pointOri.x = rand() % 10;
        pointOri.y = rand() % 10;
        pointOri.z = rand() % 10;

        laser1.push_back(pointOri);

        float angle = 30.0 * M_PI / 180.0;
        pointSel.x = cos(angle) * pointOri.x - sin(angle) * pointOri.y + 1.0;
        pointSel.y = sin(angle) * pointOri.x + cos(angle) * pointOri.y + 2.0;
        pointSel.z = pointOri.z + 1.0;

        laser2.push_back(pointSel);
    }

    Eigen::Matrix4f guess;
    pcl::registration::TransformationEstimationLM<pcl::PointXYZ, pcl::PointXYZ, float> lm;
    lm.estimateRigidTransformation(laser1, laser2, guess);

    std::cout << guess << std::endl;

    return 0;
}

