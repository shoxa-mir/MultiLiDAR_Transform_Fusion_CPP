#include "utils.h"
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/passthrough.h>
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
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_transformed;
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_filtered;
    pcl::PointCloud<pcl::PointXYZI>::Ptr mergedCloud;

    pcl::PassThrough<pcl::PointXYZI> pass;

    ros::Publisher pubMerged;
    ros::Publisher pubTransformed;
    
    Eigen::Vector3f translation;
    Eigen::Quaternionf rotation;

    std_msgs::Header cloudHeader;
    string frameID_;
    int idx = 0;

    std::mutex merge_mutex;

public:
    // void cloudFilter(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud)
    // {
    //     pass.setInputCloud(cloud);
    //     pass.setFilterFieldName("x");
    //     pass.setFilterLimits(-5.5, 0);
    //     pass.filter(*cloud_filtered);
    // }

    void copyPointCloud(const sensor_msgs::PointCloud2& laserCloudMsg, const std::string& id)
    {
        pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);
        pcl::fromROSMsg(laserCloudMsg, *cloud);
        
        Eigen::Affine3f transform = Eigen::Affine3f::Identity();

        if (id == "front_l") {
            translation << -2.1f, 1.0f, 0.0f;
            rotation = Eigen::AngleAxisf(M_PI, Eigen::Vector3f::UnitZ());
        } else if (id == "front_r") {
            translation << -2.1f, -1.0f, 0.0f;
            rotation = Eigen::Quaternionf::Identity();
        } else if (id == "rear_l") {
            translation << -8.8f, 1.0f, 0.0f;
            rotation = Eigen::AngleAxisf(-M_PI*5/6, Eigen::Vector3f::UnitZ());
            // rotation = Eigen::AngleAxisf(-M_PI*3/4, Eigen::Vector3f::UnitZ());
        } else if (id == "rear_r") {
            translation << -8.8f, -1.0f, 0.0f;
            rotation = Eigen::AngleAxisf(-M_PI/6, Eigen::Vector3f::UnitZ());
        } else {
            translation << 0.0f, 0.0f, 0.0f;
            rotation = Eigen::AngleAxisf(M_PI/2, Eigen::Vector3f::UnitZ());
        }

        transform.translation() = translation;
        transform.rotate(rotation);

        pcl::transformPointCloud(*cloud, *cloud_transformed, transform);

        if (id == "front_l" || id == "front_r") {
            pass.setInputCloud(cloud_transformed);
            pass.setFilterFieldName("x");
            pass.setFilterLimits(-5.5, 0);
            pass.filter(*cloud_filtered);
            *transformedCloud = *cloud_filtered;
        } else if (id == "rear_l" || id == "rear_r") {
            pass.setInputCloud(cloud_transformed);
            pass.setFilterFieldName("x");
            pass.setFilterLimits(-50, -5.5);
            pass.filter(*cloud_filtered);

            pass.setInputCloud(cloud_filtered);
            pass.setFilterFieldName("y");
            if (id == "rear_r") {
                pass.setFilterLimits(-50, 0);
            } else {
                pass.setFilterLimits(0, 50);
            }
            pass.filter(*cloud_filtered);

            *transformedCloud = *cloud_filtered;
        } else {
            *transformedCloud = *cloud_transformed;
        }

        cloudHeader.stamp = ros::Time::now();
        cloudHeader.frame_id = laserCloudMsg.header.frame_id;
        {
            std::lock_guard<std::mutex> lock(merge_mutex);
            *mergedCloud += *transformedCloud;
            idx++;
        }
    }


    void cloudPublish()
    {
        sensor_msgs::PointCloud2 laserCloudTemp;
        if (pubTransformed.getNumSubscribers() != 0)
        {
            pcl::toROSMsg(*transformedCloud, laserCloudTemp);
            laserCloudTemp.header.stamp = cloudHeader.stamp;
            laserCloudTemp.header.frame_id = frameID;
            pubTransformed.publish(laserCloudTemp);
        }
        
        if (pubMerged.getNumSubscribers() > 0 & idx%5==0) {
            sensor_msgs::PointCloud2 output;
            pcl::toROSMsg(*mergedCloud, output);
            output.header.stamp = ros::Time::now();
            output.header.frame_id = frameID;
            pubMerged.publish(output);
            // mergedCloud->clear();
            ROS_INFO_STREAM("Merged cloud now has " << mergedCloud->points.size() << " points.");
            mergedCloud.reset(new pcl::PointCloud<pcl::PointXYZI>());
            // ROS_INFO_STREAM("Reset " << mergedCloud->points.size() << " points.");
        }
    }


    void allocateMemory()
    {
        transformedCloud.reset(new pcl::PointCloud<pcl::PointXYZI>());
        mergedCloud.reset(new pcl::PointCloud<pcl::PointXYZI>());
        cloud_transformed.reset(new pcl::PointCloud<pcl::PointXYZI>());
        cloud_filtered.reset(new pcl::PointCloud<pcl::PointXYZI>());
    }
    

    void pointCloudCallback(const sensor_msgs::PointCloud2 msg) {
        copyPointCloud(msg, msg.header.frame_id.c_str());
        cloudPublish();
    }


    void setupSubscribers() {
        subpointCloudTopic_fc = nh.subscribe<sensor_msgs::PointCloud2>(pointCloudTopic_fc, 10, &Process::pointCloudCallback, this);
        // subpointCloudTopic_ft = nh.subscribe<sensor_msgs::PointCloud2>(pointCloudTopic_ft, 10, &Process::pointCloudCallback, this);
        subpointCloudTopic_fr = nh.subscribe<sensor_msgs::PointCloud2>(pointCloudTopic_fr, 10, &Process::pointCloudCallback, this);
        subpointCloudTopic_fl = nh.subscribe<sensor_msgs::PointCloud2>(pointCloudTopic_fl, 10, &Process::pointCloudCallback, this);
        // subpointCloudTopic_rr = nh.subscribe<sensor_msgs::PointCloud2>(pointCloudTopic_rr, 10, &Process::pointCloudCallback, this);
        // subpointCloudTopic_rl = nh.subscribe<sensor_msgs::PointCloud2>(pointCloudTopic_rl, 10, &Process::pointCloudCallback, this);
    }


    void setupPublishers() {
        pubMerged = nh.advertise<sensor_msgs::PointCloud2>("pandar", 1);
        // pubMerged = nh.advertise<sensor_msgs::PointCloud2>(pubMergedTopic, 1);
        pubTransformed = nh.advertise<sensor_msgs::PointCloud2>(pubTransformedTopic, 1);
    }


    Process() : nh("~") {
        setupSubscribers();
        setupPublishers();
        allocateMemory();
        ROS_INFO("Started Point Cloud Processing Node.");
    }

    ~Process() {}

};


int main(int argc, char **argv) {
    // ros::init(argc, argv, "lidar_fusion");
    ros::init(argc, argv, "hesai");
    while(ros::ok()) {
        Process P;
        ros::spin();
    }
    return 0;
}