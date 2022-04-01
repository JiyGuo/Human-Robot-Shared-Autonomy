#include "readImage/readImage.h"
#include<opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/core/core.hpp>
#include "robotController/robotController.h"
using namespace std;
using namespace cv;
const int CHESS_Row=8;
const int  CHESS_Column=11;
int main(int argc, char** argv){

    ros::init(argc, argv, "Test");
    ros::NodeHandle *nh;
    nh = new ros::NodeHandle();
    readImage myReader(nh);
    robotController myController(nh);

    ros::AsyncSpinner spinner(2);
    spinner.start();
    ros::Duration(3).sleep();
	//VideoCapture cap(0);
	Mat frame;
	vector<Point2f> points;

	Mat chess_image_ori;
	myReader.getOneFrameRealSenseImage(chess_image_ori);
	
	cv::findChessboardCorners(chess_image_ori, Size(8, 11), points, cv::CALIB_CB_ADAPTIVE_THRESH);
	Mat show_chessboard = chess_image_ori.clone();
	
	bool ischessfound=false;
	cv::drawChessboardCorners(chess_image_ori, Size(8, 11), points, ischessfound);
	cout<<"point_size: "<<points.size()<<endl;
//	imshow("chess", chess_image_ori);
	for (int i = 0; i < points.size(); i++)
	{
		circle(show_chessboard, points[i], 5, Scalar(255, 0, 0), 1, 4, 0);
		putText(show_chessboard, to_string(i), points[i], cv::FONT_HERSHEY_PLAIN, 1, Scalar(255, 255, 0), 1, 8);
		cout << points[i].x <<" "<< points[i].y << endl;
	}
//	imshow("chessOrder", show_chessboard);
	imwrite("/home/jiyguo/JTDQ/ur_ws/chess_order.png", show_chessboard);
//    waitKey(0);

    KDL::Frame curFrame;
    std::vector<vector<double>> points_in_robot;
    points_in_robot.clear();
    std::vector<double> cur_point;

    for (int i = 0; i < 3; ++i){
        cur_point.clear();
        cout<<"Drag the robot tool to cross point and then Press Enter."<<endl;
//        cin.get();
        while (getchar() != '\n');
        myController.printCurJoint();
        myController.getRobotFrame(curFrame);
        cur_point.emplace_back(curFrame.p.x());
        cur_point.emplace_back(curFrame.p.y());
        cout<<cur_point[0]<<" , "<<cur_point[1]<<endl;
        points_in_robot.emplace_back(cur_point);
    }

/*
    cur_point.emplace_back(0.725681);
    cur_point.emplace_back(-0.065886);
    cout<<cur_point[0]<<" , "<<cur_point[1]<<endl;
    points_in_robot.emplace_back(cur_point);
    cur_point.clear()
    cur_point.emplace_back(0.551391);
    cur_point.emplace_back(-0.0667253);
    cout<<cur_point[0]<<" , "<<cur_point[1]<<endl;
    points_in_robot.emplace_back(cur_point);

    cur_point.emplace_back(0.729073);
    cur_point.emplace_back(0.182845);
    cout<<cur_point[0]<<" , "<<cur_point[1]<<endl;
    points_in_robot.emplace_back(cur_point);
*/

    Eigen::Vector2d delta_s,delta_l,tmp_points;
    Eigen::Vector2d ori_point(points_in_robot[0].at(0),points_in_robot[0].at(1));
    Eigen::Vector2d point_S(points_in_robot[1].at(0),points_in_robot[1].at(1));
    Eigen::Vector2d point_L(points_in_robot[2].at(0),points_in_robot[2].at(1));

    delta_s = (point_S-ori_point)/(CHESS_Row-1);
    delta_l = (point_L-ori_point)/(CHESS_Column-1);

    Eigen::Matrix<double,3,CHESS_Row*CHESS_Column> robot_pos;
    Eigen::Matrix<double,3,CHESS_Row*CHESS_Column> pixel_pos;
    pixel_pos.setOnes();
    robot_pos.setOnes();
    for (int i = 0; i < CHESS_Column; ++i) {
        tmp_points = ori_point + i * delta_l;
        for (int j = 0; j < CHESS_Row; ++j) {
            robot_pos.block<2,1>(0,CHESS_Row*i+j) = tmp_points ;
            tmp_points = tmp_points + delta_s;
        }
    }

    for (int i = 0; i < points.size(); ++i) {
        pixel_pos(0,i) = points[i].x ;
        pixel_pos(1,i) = points[i].y ;
    }
    Eigen::Matrix3d calibration_matrix;
    cout<<"robot_pos"<<robot_pos<<endl;
    cout<<"pixel_pos"<<pixel_pos<<endl;

    calibration_matrix=robot_pos * pixel_pos.transpose()*(pixel_pos * pixel_pos.transpose()).inverse();
    for (int i = 0; i < calibration_matrix.rows(); ++i) {
        for (int j = 0; j < calibration_matrix.cols(); ++j) {
            cout<< setw(6) << calibration_matrix(i,j)<<" ";
        }
        cout<<endl;
    }
	return 0;
}
