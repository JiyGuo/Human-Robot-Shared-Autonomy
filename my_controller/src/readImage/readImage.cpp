//
// Created by root on 8/23/20.
//

#include "readImage/readImage.h"
#include<opencv2/opencv.hpp>
#include <pcl_conversions/pcl_conversions.h>

readImage::readImage(ros::NodeHandle* node)
{
    n=node;

    realSenseImageSub=n->subscribe("/camera/color/image_raw",10,&readImage::realSenseImageCallback,this);
    realSenseDepthSub=n->subscribe("/camera/aligned_depth_to_color/image_raw",10,&readImage::realSenseDepthCallback,this);
//    realSensePointSub=n->subscribe(realsense_pcl_topic,10,&readImage::realSensePointCallback,this);

    generateService=n->advertiseService("/callImage",&readImage::generatrCallback,this);

    realsenseImageFlag=false;
    realsenseImageFinishFlag=false;
    realsenseDepthFlag=false;
    realsenseDepthFinishFlag=false;
    realsensePointFlag=false;
    realsensePointFinishFlag=false;

//    spinner = new ros::AsyncSpinner(2);
//    spinner->start();
//    ros::Duration(4.0).sleep();
}

void readImage::realSenseImageCallback(const sensor_msgs::Image::ConstPtr& msgs)
{
    if(realsenseImageFlag)
    {
        realSenseImage=msgs;
        realsenseImageFlag=false;
        cv_bridge::CvImagePtr cv_ptr;
        cv_ptr = cv_bridge::toCvCopy(realSenseImage, sensor_msgs::image_encodings::BGR8);
        cv_ptr->image.copyTo(realrenseImageMat);
        realsenseImageFinishFlag=true;
    }
}

void readImage::realSenseDepthCallback(const sensor_msgs::Image::ConstPtr& msgs)
{
    if(realsenseDepthFlag)
    {
        realSenseDepth=msgs;
        realsenseDepthFlag=false;
        cv_bridge::CvImagePtr cv_ptr;
//        cv_ptr = cv_bridge::toCvCopy(realSenseDepth, sensor_msgs::image_encodings::TYPE_32FC1);
        cv_ptr = cv_bridge::toCvCopy(realSenseDepth, sensor_msgs::image_encodings::TYPE_16UC1);
        cv_ptr->image.copyTo(realrenseDepthMat);
//        std::cout<<"realrenseDepthMat.rows: "<<realrenseDepthMat.rows<<std::endl;
        realsenseDepthFinishFlag=true;
    }

}

void readImage::realSensePointCallback(const sensor_msgs::PointCloud2::ConstPtr& msgs)
{
    if(realsensePointFlag)
    {
        realSensePoint=msgs;
        realsensePointFlag=false;
        pcl::PCLPointCloud2 cloud2;
        pcl_conversions::toPCL(*realSensePoint, cloud2);
        pcl::fromPCLPointCloud2(cloud2,realsensePointCloud);
        realsensePointFinishFlag=true;
    }
}

bool readImage::generatrCallback(std_srvs::Trigger::Request  &req,std_srvs::Trigger::Response &res)
{
    ROS_INFO("recvice call");
    cv::Mat srcimage;
    pcl::PointCloud<pcl::PointXYZ> srccloud;
//    getOneFrameKinectImage(srcimage);
    //cv::imshow("img",srcimage);
    //cv::waitKey();
    getOneFrameRealSenseImage(srcimage);
    cv::imwrite("/root/ocrtoc_ws/src/rgb.png",srcimage);
    getOneFrameRealSenseDepth(srcimage);
    cv::imwrite("/home/jiyguo/JTDQ/data/depth.png",srcimage);
//    getOneFrameRealSensePoint(srccloud);
//    pcl::io::savePCDFileASCII("/root/ocrtoc_ws/src/realsense.pcd",srccloud);
    ROS_INFO("finish");
    res.success = true;
    res.message = "read img";
    return true;
}

void readImage::getOneFrameRealSenseImage(cv::Mat& out)
{
    ROS_INFO("recvice realsense image call");
    realsenseImageFlag=true;
    while(1)
    {
        ros::spinOnce();
        if(realsenseImageFinishFlag)
        {
            break;
        }
    }
    realsenseImageFinishFlag=false;
    out=realrenseImageMat;
}

void readImage::getOneFrameRealSenseDepth(cv::Mat& out)
{
    ROS_INFO("recvice realsense depth call");
    realsenseDepthFlag=true;
    while(1)
    {
        ros::spinOnce();
        if(realsenseDepthFinishFlag)
        {
            break;
        }
    }
    realsenseDepthFinishFlag=false;
    out=realrenseDepthMat;
}

void readImage::getOneFrameRealSensePoint(pcl::PointCloud<pcl::PointXYZ>& out)
{
    ROS_INFO("recvice realsense point call");
    realsensePointFlag=true;
    while(1)
    {
        ros::spinOnce();
        if(realsensePointFinishFlag)
        {
            break;
        }
    }
    realsensePointFinishFlag=false;
    out=realsensePointCloud;
}

readImage::~readImage(){
//    delete spinner;
    delete n;
}

