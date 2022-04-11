//
// Tutorial Author: shapelim@kaist.ac.kr (임형태)
#include <ros/package.h>
#include <std_msgs/Int8.h>
#include "ros/ros.h" // ROS Default Header File

#include <stdio.h>
#include <iostream>
#include <string>

#include <pcl/point_types.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/common/transforms.h>
#include <pcl/registration/icp.h>
#include <chrono>

#include <pcl_ros/point_cloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/LaserScan.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>

#include "laser_geometry/laser_geometry.h"
#include "sensor_msgs/PointCloud.h"
#include "sensor_msgs/point_cloud2_iterator.h"
#include <math.h>

#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseWithCovariance.h>
#include <geometry_msgs/Pose.h>

#include <ignition/math.hh>

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>
using namespace std;

pcl::PointCloud<pcl::PointXYZ>::Ptr src;
pcl::PointCloud<pcl::PointXYZ>::Ptr tgt;

laser_geometry::LaserProjection projector_;

geometry_msgs::Point ag_position;
double Q_angles[4];

void msgCallback_pose(const nav_msgs::Odometry::ConstPtr &msg)
{
    ag_position = msg->pose.pose.position;

    Q_angles[0] = msg->pose.pose.orientation.x;
    Q_angles[1] = msg->pose.pose.orientation.y;
    Q_angles[2] = msg->pose.pose.orientation.z;
    Q_angles[3] = msg->pose.pose.orientation.w;
}

void msgCallback_laser(const sensor_msgs::LaserScan::ConstPtr &msg)
{
    sensor_msgs::PointCloud2 cloud;
    projector_.projectLaser(*msg, cloud);
    pcl::fromROSMsg(cloud, *src);
}

pcl::PointCloud<pcl::PointXYZ>::ConstPtr load_bin(const string &filename)
{
    FILE *file = fopen(filename.c_str(), "rb");
    if (!file)
    {
        std::cerr << "Error: failed to load " << filename << std::endl;
        return nullptr;
    }
    std::vector<float> buffer(1000000);
    size_t num_points =
        fread(reinterpret_cast<char *>(buffer.data()), sizeof(float), buffer.size(), file) / 4;
    fclose(file);

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
    cloud->resize(num_points);

    for (int i = 0; i < num_points; i++)
    {
        auto &pt = cloud->at(i);
        pt.x = buffer[i * 4];
        pt.y = buffer[i * 4 + 1];
        pt.z = buffer[i * 4 + 2];
        // pt.intensity = buffer[i * 4 + 3];
    }

    return cloud;
}

void colorize(const pcl::PointCloud<pcl::PointXYZ> &pc,
              pcl::PointCloud<pcl::PointXYZRGB> &pc_colored,
              const std::vector<int> &color)
{

    int N = pc.points.size();

    pc_colored.clear();
    pcl::PointXYZRGB pt_tmp;
    for (int i = 0; i < N; ++i)
    {
        const auto &pt = pc.points[i];
        pt_tmp.x = pt.x;
        pt_tmp.y = pt.y;
        pt_tmp.z = pt.z;
        pt_tmp.r = color[0];
        pt_tmp.g = color[1];
        pt_tmp.b = color[2];
        pc_colored.points.emplace_back(pt_tmp);
    }
}

int main(int argc, char **argv)
{
    src.reset(new pcl::PointCloud<pcl::PointXYZ>());
    tgt.reset(new pcl::PointCloud<pcl::PointXYZ>());

    ros::init(argc, argv, "hero_pcl"); // Initializes Node Name
    ros::NodeHandle
        nh;                 // Node handle declaration for communication with ROS system
    ros::Rate loop_rate(1); // Hz

    ros::Subscriber sub_state =
        nh.subscribe("/hero_agent/laser_scan", 100, msgCallback_laser);
    ros::Publisher cloud_pub;
    cloud_pub = nh.advertise<sensor_msgs::PointCloud2>("/cloud_all", 50);

    ros::Subscriber sub_pose = nh.subscribe("/hero_agent/pose_gt", 100, msgCallback_pose);

    Eigen::Matrix4f trans;

    while (ros::ok())
    {
        trans << 1, 0, 0, ag_position.x,
            0, 1, 0, ag_position.y,
            0, 0, 1, ag_position.z,
            0, 0, 0, 1;
        pcl::transformPointCloud(*tgt, *tgt, trans);
        tgt->points.emplace_back(*src);
        sensor_msgs::PointCloud2 cloudmsg;
        pcl::toROSMsg(*tgt, cloudmsg);
        cloudmsg.header.frame_id = "/hero_agent/base_link";
        cloud_pub.publish(cloudmsg);

        loop_rate.sleep();
        ros::spinOnce();
    }
}