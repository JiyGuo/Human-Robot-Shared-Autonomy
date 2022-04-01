//
// Created by jiyguo on 20-10-25.
//
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/opencv.hpp>
#include <vector>
#include <string>

#include <cv_bridge/cv_bridge.h>
#include "readImage/readImage.h"
#include "robotController/robotController.h"

using namespace cv;
using namespace std;
void detect(Mat ori_image , vector<Point2f>& pos);
int main(int argc, char** argv){
    ros::init(argc, argv, "Test");
    ros::NodeHandle *nh;
    nh = new ros::NodeHandle();
    robotController myController(nh);
    readImage myReader(nh);

    ros::AsyncSpinner spinner(2);
    spinner.start();
    ros::Duration(3).sleep();

    myController.moveToJoint(myController.homeJoint,5);
    ros::Duration(1).sleep();

    cv::Mat rgb_img;
    myReader.getOneFrameRealSenseImage(rgb_img);
    std::string rgb = "/home/jiyguo/JTDQ/data/rgb.png";
    cv::imwrite(rgb,rgb_img);

    vector<Point2f> pos;
    detect(rgb_img, pos);


    KDL::Frame initFrame;
    KDL::Frame targetFrame;
    targetFrame.M = KDL::Rotation::RotX(M_PI);
    targetFrame.p = KDL::Vector(0.45,0.3,0.05);
    initFrame.M = KDL::Rotation::RotX(M_PI);

    for (int i = 0; i < pos.size(); ++i) {
        initFrame.p = KDL::Vector(pos[i].x,pos[i].y,0.05);
        targetFrame.p.x(targetFrame.p.x()+0.02);
        myController.grasp(initFrame,targetFrame);
    }
}


void detect(Mat ori_image , vector<Point2f>& pos){

    //imshow("原图", ori_image);
    Rect rect(150, 55, 320, 270);//左上坐标（x,y）和矩形的长(x)宽(y)

    //cv::rectangle(ori_image, rect, Scalar(255, 0, 0), 1, LINE_8, 0);
    //imshow("裁切位置", ori_image);
    GaussianBlur(ori_image, ori_image,Size(3,3),3,3);
    Mat img_crop = ori_image(rect);
    //imshow("裁切后", img_crop);
    std::string rgb_crop = "/home/jiyguo/JTDQ/data/rgb_crop.png";
    cv::imwrite(rgb_crop,img_crop);
    Mat gray_image, bin_image;
    cvtColor(img_crop, gray_image, CV_BGR2GRAY);
    threshold(gray_image, bin_image, 60, 255, CV_THRESH_BINARY);
//    Mat fhsv;
//    cvtColor(img_crop,fhsv,COLOR_BGR2HSV);   //将图像转换为HSV模型
//    int minh,maxh,mins,maxs,minv,maxv;
//    minh = 26;
//    maxh = 35;
//    mins = 43;
//    maxs = 255;
//    minv = 46;
//    maxv = 255;
//    inRange(fhsv,Scalar(minh,mins,minv),Scalar(maxh,maxs,maxv),bin_image);          //找寻在要求区间内的颜色
//    imshow("ABC",bin_image);
    imshow("bin", bin_image);
    waitKey();
    std::string rgb_bin = "/home/jiyguo/JTDQ/data/rgb_bin.png";
    cv::imwrite(rgb_bin,bin_image);
    vector<vector<Point> > contours;
    vector<Vec4i> hierarchy;
    findContours(bin_image, contours,hierarchy, CV_RETR_LIST, CHAIN_APPROX_SIMPLE, Point(0, 0));
    vector<Moments> mu(contours.size());
    for (int i = 0; i < contours.size(); i++)
    {
        mu[i] = moments(contours[i], false);
    }
    ///  计算中心矩:
    vector<Point2f> mc(contours.size());
    vector<Point2f> ori_point;
    double area;
    for (int i = 0; i < contours.size(); i++)
    {
        mc[i] = Point2f(mu[i].m10 / mu[i].m00, mu[i].m01 / mu[i].m00);
        area = cv::contourArea(contours[i]);
        if(area<700&&area>100)
            ori_point.emplace_back(mc[i]);
    }
    Mat drawing = Mat::zeros(bin_image.size(), CV_8UC1);
    for (int i = 0; i < contours.size(); i++)
    {
        Scalar color = Scalar(255);
        drawContours(drawing, contours, i, color, 2, 8, hierarchy, 0, Point());
        circle(drawing, mc[i], 4, color, -1, 8, 0);
    }
    //imshow("outImage", img_crop);


    Eigen::Matrix3d world2cam;
    world2cam <<-1.83833e-05, -0.00115201, 0.810336,
            -0.00115027, -1.54878e-05, 0.444133,
            0,      0,       1;
    Eigen::Vector3d pos_cam;
    Eigen::Vector3d pos_world;
    pos.resize(ori_point.size());
    for (int i = 0; i < ori_point.size(); ++i) {
        ori_point[i].x += 150;
        ori_point[i].y += 55;
        circle(ori_image,ori_point[i],4, Scalar(255,0,0), -1, 8, 0);
        cout << ori_point[i].x<<","<<ori_point[i].y << endl;
        pos_cam<<ori_point[i].x,ori_point[i].y,1.0;

        pos_world = world2cam*pos_cam;
        cout<<pos_world(0)<<" , "<<pos_world(1)<<endl;
        pos[i].x = pos_world(0);
        pos[i].y = pos_world(1);
    }
    std::string rgb = "/home/jiyguo/JTDQ/data/rgb_pos.png";
    cv::imwrite(rgb,ori_image);
    imshow("out_all", ori_image);
    waitKey();
}