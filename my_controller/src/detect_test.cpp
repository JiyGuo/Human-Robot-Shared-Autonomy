//
// Created by jiyguo on 2020/11/29.
//

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
void showHistgram(Mat gray_image);
void detect(Mat ori_image , vector<Point2f>& pos);

int main(int argc, char** argv){
    ros::init(argc, argv, "Test");
    ros::NodeHandle *nh;
    nh = new ros::NodeHandle();
    readImage myReader(nh);

    ros::AsyncSpinner spinner(2);
    spinner.start();
    ros::Duration(3).sleep();

    cv::Mat rgb_img;
    int if_read = 0;
    cout<<"if_read? :"<<endl;
    cin>>if_read;
    if (if_read){
        myReader.getOneFrameRealSenseImage(rgb_img);
        imshow("source",rgb_img);
        imwrite("/home/jiyguo/JTDQ/data/source.png",rgb_img);
    }else{
        rgb_img = imread("/home/jiyguo/JTDQ/data/source.png");
    }
    vector<Point2f> pos;
    detect(rgb_img, pos);

}


void detect(Mat ori_image , vector<Point2f>& pos){

    //imshow("原图", ori_image);
    Rect rect(150, 55, 320, 270);//左上坐标（x,y）和矩形的长(x)宽(y)

    //cv::rectangle(ori_image, rect, Scalar(255, 0, 0), 1, LINE_8, 0);
    //imshow("裁切位置", ori_image);
    GaussianBlur(ori_image, ori_image,Size(3,3),3,3);
//    Mat img_crop = ori_image(rect);
//    imshow("crop", img_crop);
//    waitKey();

    Mat img_crop = ori_image;
//    std::string rgb_crop = "/home/jiyguo/JTDQ/data/rgb_crop.png";
//    cv::imwrite(rgb_crop,img_crop);
    Mat gray_image, bin_image;
    cvtColor(img_crop, gray_image, CV_BGR2GRAY);
    showHistgram(gray_image);

//    imshow("gray_image", gray_image);
//    waitKey();
    threshold(gray_image, bin_image, 103 , 255, CV_THRESH_BINARY_INV);
//    threshold(gray_image, bin_image, 103 , 255, CV_THRESH_OTSU);

    imshow("bin", bin_image);
    waitKey();
    imwrite("/home/jiyguo/JTDQ/data/bin.png",bin_image);
    Mat element = getStructuringElement(MORPH_RECT, Size(10, 10));
    dilate(bin_image,bin_image, element);
    imshow("dilate", bin_image);
    waitKey(0);
    //进行腐蚀操作
    element = getStructuringElement(MORPH_RECT, Size(12, 12));
    erode(bin_image,bin_image, element);
    //显示效果图
    imshow("erode", bin_image);
    waitKey(0);
    imwrite("/home/jiyguo/JTDQ/data/erode.png",bin_image);

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

//    std::string rgb_bin = "/home/jiyguo/JTDQ/data/rgb_bin.png";
//    cv::imwrite(rgb_bin,bin_image);


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
        cout<<"area: "<<area<<endl;
        if(area>200)
            ori_point.emplace_back(mc[i]);
    }
    Mat drawing = Mat::zeros(bin_image.size(), CV_8UC1);
//    for (int i = 0; i < contours.size(); i++)
//    {
//        Scalar color = Scalar(255);
//        drawContours(drawing, contours, i, color, 2, 8, hierarchy, 0, Point());
//        circle(drawing, mc[i], 4, color, -1, 8, 0);
//    }
//    imshow("outImage", drawing);
//    waitKey();
    cout<<ori_point.size()<<endl;
    for (int i = 0; i < ori_point.size(); ++i) {
//        ori_point[i].x += 150;
//        ori_point[i].y += 55;
        circle(ori_image,ori_point[i],5, Scalar(0,0,255), -1, 8, 0);
    }
//    std::string rgb = "/home/jiyguo/JTDQ/data/rgb_pos.png";
//    cv::imwrite(rgb,ori_image);

    //寻找最小外包矩形
    RotatedRect minRect;
    Point2f fourPoint2f[4];
    for (int i = 0; i < contours.size(); ++i) {
        minRect = minAreaRect(contours[i]);
        cout<<"minRect:"<<minRect.angle<<", "<<minRect.center.x<<", "<<minRect.center.y<<", "<<endl;
        //将minRect的四个顶点坐标值放到fourPoint的数组中
        minRect.points(fourPoint2f);

        //根据得到的四个点的坐标  绘制矩形
        for (int j = 0; j < 3; j++)
        {
            line(ori_image, fourPoint2f[j], fourPoint2f[j + 1], Scalar(0,0,255), 3);
        }
        line(ori_image, fourPoint2f[0], fourPoint2f[3], Scalar(0, 0, 255), 3);
    }
    imshow("out_all", ori_image);
    imwrite("/home/jiyguo/JTDQ/data/detect.png",ori_image);
    waitKey();

}

void showHistgram(Mat gray_image){
    MatND hist;
    float hranges[2];
    hranges[0] = 0.0f;
    hranges[1] = 255.0f;
    const float* ranges[1];
    ranges[0] = hranges;
    int channels[1];
    channels[0] = 0;
    int histSize[1];
    histSize[0] = 256;
    calcHist( &gray_image, 1,
            channels, Mat(),
            hist, 1, histSize,
            ranges );

    // 最大值，最小值
    double maxVal = 0.0f;
    double minVal = 0.0f;
    minMaxLoc(hist, &minVal, &maxVal);
    //显示直方图的图像
    Mat histImg(histSize[0], histSize[0], CV_8U, Scalar(255));

    // 设置最高点为nbins的90%
    int hpt = static_cast<int>(0.9 * histSize[0]);
    //每个条目绘制一条垂直线
    for (int h = 0; h < histSize[0]; h++)
    {
        float binVal = hist.at<float>(h);
        int intensity = static_cast<int>(binVal * hpt / maxVal);
        // 两点之间绘制一条直线
        line(histImg, Point(h, histSize[0]), Point(h, histSize[0] - intensity), Scalar::all(0));
    }
    imshow("histImg", histImg);
    waitKey();
}