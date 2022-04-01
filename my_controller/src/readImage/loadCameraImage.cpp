#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include "readImage/readImage.h"
#include "string"

using namespace std;
int main(int argc,char** argv)
{
    ros::init(argc, argv, "loadCameraImage");
    ros::NodeHandle* n = new ros::NodeHandle();
    readImage m(n);

    int i;
    cin>>i;
    ROS_INFO("recvice call");
    cv::Mat srcimage;
    std::string rgb = "/home/jiyguo/JTDQ/data/rgb"+to_string(i)+".png";
    std::string dpt = "/home/jiyguo/JTDQ/data/dpt"+to_string(i)+".png";
    m.getOneFrameRealSenseImage(srcimage);
    cv::imwrite(rgb,srcimage);
    m.getOneFrameRealSenseDepth(srcimage);
    cv::imwrite(dpt,srcimage);
    ROS_INFO("finish");
//    delete n;
//    ros::spin();
    return 0;
}