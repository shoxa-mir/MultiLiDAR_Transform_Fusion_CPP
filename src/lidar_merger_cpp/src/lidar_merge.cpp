#include "utils.h"
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <chrono>
#include <thread>
#include <math.h>
#include <typeinfo>
#include <iomanip>
#include <ros/ros.h>
#include <std_msgs/String.h>

class Process {
private:
    ros::NodeHandle nh;

    ros::Subscriber subpointCloudTopic_fc;
    ros::Subscriber subpointCloudTopic_ft;
    ros::Subscriber subpointCloudTopic_fr;
    ros::Subscriber subpointCloudTopic_fl;
    ros::Subscriber subpointCloudTopic_rr;
    ros::Subscriber subpointCloudTopic_rl;

    pcl::PointCloud<pcl::PointXYZI>::Ptr transformedCloud;
    pcl::PointCloud<pcl::PointXYZI>::Ptr mergedCloud;

    ros::Publisher pubMerged;
    ros::Publisher pubTransformed;
    
    std_msgs::Header cloudHeader;

public:
    void pointCloudCallback(const sensor_msgs::PointCloud2 msg) {
        ROS_INFO("Received a point cloud message");
        ROS_INFO("Frame ID: %s", msg.header.frame_id.c_str());
        copyPointCloud(msg, msg.header.frame_id.c_str());
        cloudPublish();

    }
    
    void copyPointCloud(const sensor_msgs::PointCloud2 laserCloudMsg, string id)
    {
        pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);
        pcl::fromROSMsg(laserCloudMsg, *cloud);

        Eigen::Vector3f translation(0.0f, 0.0f, 1.4f);
        Eigen::Quaternionf rotation(0.7071f, 0.0f, 0.0f, 0.7071f);
        
        if (id=="front64") {
            Eigen::Vector3f translation(1.2f, 2.5f, -0.4f);
            Eigen::Quaternionf rotation(0.7071f, 0.0f, 0.0f, 0.7071f);
        }

        
        Eigen::Affine3f transform = Eigen::Affine3f::Identity();
        transform.translation() = translation;
        transform.rotate(rotation);

        pcl::transformPointCloud(*cloud, *cloud, transform);

        cloudHeader.stamp = ros::Time::now();
        cloudHeader = laserCloudMsg.header;

        *transformedCloud = *cloud; // Convert pcl::PointXYZ to pcl::PointXYZI
    }


    void cloudPublish()
    {
        sensor_msgs::PointCloud2 laserCloudTemp;
        // projected transformed cloud
        if (pubTransformed.getNumSubscribers() != 0)
        {
            pcl::toROSMsg(*transformedCloud, laserCloudTemp);
            laserCloudTemp.header.stamp = cloudHeader.stamp;
            laserCloudTemp.header.frame_id = frameID;
            pubTransformed.publish(laserCloudTemp);
        }
    }


    void allocateMemory()
    {
        transformedCloud.reset(new pcl::PointCloud<pcl::PointXYZI>());
        mergedCloud.reset(new pcl::PointCloud<pcl::PointXYZI>());
    }

    Process() : nh("~") { 
        // subpointCloudTopic_fc = nh.subscribe<sensor_msgs::PointCloud2>(pointCloudTopic_fc, 10, &Process::pointCloudCallback, this);
        subpointCloudTopic_ft = nh.subscribe<sensor_msgs::PointCloud2>(pointCloudTopic_ft, 10, &Process::pointCloudCallback, this);
        subpointCloudTopic_fr = nh.subscribe<sensor_msgs::PointCloud2>(pointCloudTopic_fr, 10, &Process::pointCloudCallback, this);
        subpointCloudTopic_fl = nh.subscribe<sensor_msgs::PointCloud2>(pointCloudTopic_fl, 10, &Process::pointCloudCallback, this);
        subpointCloudTopic_rr = nh.subscribe<sensor_msgs::PointCloud2>(pointCloudTopic_rr, 10, &Process::pointCloudCallback, this);
        subpointCloudTopic_rl = nh.subscribe<sensor_msgs::PointCloud2>(pointCloudTopic_rl, 10, &Process::pointCloudCallback, this);
        
        pubMerged = nh.advertise<sensor_msgs::PointCloud2>(pubMergedTopic, 1);
        pubTransformed = nh.advertise<sensor_msgs::PointCloud2>(pubTransformedTopic, 1);

        ROS_INFO("Start fusion");

        allocateMemory();
    }

    ~Process() {}

};


int main(int argc, char **argv) {
    ros::init(argc, argv, "lidar_fusion");
    while(ros::ok()) {
        Process P;
        ros::spin();
    }
    return 0;
}