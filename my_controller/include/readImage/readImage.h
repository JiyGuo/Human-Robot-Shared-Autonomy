//
// Created by root on 8/23/20.
//

#ifndef LOADCAMERAIMAGE_READIMAGE_H
#define LOADCAMERAIMAGE_READIMAGE_H

#include<ros/ros.h>
#include<cv_bridge/cv_bridge.h>
#include<sensor_msgs/PointCloud2.h>
#include<sensor_msgs/Image.h>
#include <std_srvs/Trigger.h>
#include<opencv2/opencv.hpp>
#include<opencv2/highgui.hpp>
#include<pcl/io/pcd_io.h>
#include<pcl/point_cloud.h>

class readImage
{
private:
    ros::Subscriber realSenseImageSub;
    ros::Subscriber realSenseDepthSub;
    ros::Subscriber realSensePointSub;
    ros::NodeHandle* n;

    sensor_msgs::Image::ConstPtr realSenseImage;
    sensor_msgs::Image::ConstPtr realSenseDepth;
    sensor_msgs::PointCloud2::ConstPtr realSensePoint;

    ros::ServiceServer generateService;

    bool realsenseImageFlag;
    bool realsenseImageFinishFlag;
    bool realsenseDepthFlag;
    bool realsenseDepthFinishFlag;
    bool realsensePointFlag;
    bool realsensePointFinishFlag;

    cv::Mat realrenseImageMat;
    cv::Mat realrenseDepthMat;
    pcl::PointCloud<pcl::PointXYZ> realsensePointCloud;

//    ros::AsyncSpinner* spinner;

private:

    void realSenseImageCallback(const sensor_msgs::Image::ConstPtr& msgs);
    void realSenseDepthCallback(const sensor_msgs::Image::ConstPtr& msgs);
    void realSensePointCallback(const sensor_msgs::PointCloud2::ConstPtr& msgs);

    bool generatrCallback(std_srvs::Trigger::Request  &req,std_srvs::Trigger::Response &res);

public:
    readImage(ros::NodeHandle* node);
    ~readImage();
    void getOneFrameRealSenseImage(cv::Mat& out);
    void getOneFrameRealSenseDepth(cv::Mat& out);
    void getOneFrameRealSensePoint(pcl::PointCloud<pcl::PointXYZ>& out);

};

#endif //LOADCAMERAIMAGE_READIMAGE_H
