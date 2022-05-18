//
// Created by zhaoxin on 18-2-26.
//

#include "geometry_msgs/PoseStamped.h"
#include "ros/ros.h"
#include "kdl/frames.hpp"
#include "myATI.h"
#include <geometry_msgs/TwistStamped.h>
#include <Eigen/Core>
#include <jntarray.hpp>
#include <sensor_msgs/JointState.h>
#include "data1.h"
#include <fstream>



using namespace KDL;
using namespace std;

const char *filename = {"/home/zhaoxin/Code/matlab/force_compesation/1.txt"};

Frame calForceFrame(Frame rigidbody)
{
    Frame end_frame;
    // 2018年03月04日数据
//    double theta =2.096718034306960;
//    double r = 0.054923927309760;

//    // 2018年03月08日数据
//    double theta =2.034326208995588;
//    double r =  0.054773511023792;
//
//
//    optiTrackFrame = Frame(Rotation::RotX(PI / 2.0)) * optiTrackFrame;
//    forceSensorFrame.M = optiTrackFrame.M * Rotation::RotX(PI / 2.0) * Rotation::RotZ(theta);
//    forceSensorFrame.p = optiTrackFrame.p - r * forceSensorFrame.M.UnitX()  + (0.028 + 0.011) * forceSensorFrame.M.UnitZ();

    double theta = 2.057601158916047;
    Frame rotztheta = Frame(Rotation::RotZ(-theta));
    Frame rotxpi = Frame(Rotation::RotX(PI));
    end_frame = rigidbody * rotztheta * rotxpi;
    end_frame.p = end_frame.p + (0.028 + 0.011) * end_frame.M.UnitZ();

//    end_frame.p = end_frame.p * 1000.0;

    return end_frame;

}

int main(int argc, char **argv) {

    ATIGravityCompensation gravityCom;
    gravityCom.initCalParam();

    ros::init(argc, argv, "calAverageForces");
    ros::NodeHandle node_handle;
    ros::AsyncSpinner spinner(1);
    spinner.start();
    ros::Rate loop_rate(240);
    ros::Publisher pub = node_handle.advertise<geometry_msgs::TwistStamped>("/CompenForce", 10);
    ros::Publisher posepub = node_handle.advertise<geometry_msgs::PoseStamped>("/endPose", 10);

    geometry_msgs::TwistStamped twist;
    geometry_msgs::PoseStamped pose;

    Eigen::Matrix<double, 6, 1> cartForce;

    float F[6];

    int index = 1;

    Frame end_frame;
    Frame optiTackFrame;

    ofstream wfile;
//    wfile.open(filename, ios::out);

    FILE *outfile = fopen(filename, "w");
    while(ros::ok())
    {
        for(int i = 0; i < 6; i++)
        {
            F[i] = (float)offlineData[index][7 + i];
        }
        optiTackFrame.p = Vector(offlineData[index][0], offlineData[index][1], offlineData[index][2]);
        optiTackFrame.M = Rotation::Quaternion(offlineData[index][3], offlineData[index][4], offlineData[index][5], offlineData[index][6]);

        end_frame = calForceFrame(optiTackFrame);

        Vector zdir = end_frame.M.UnitZ();
//        printf("z direction, %f, %f, %f \n", zdir[0], zdir[1], zdir[2]);

        cartForce = gravityCom.getCartWrench(F, end_frame.M, end_frame.p);
        twist.twist.linear.x = cartForce(0);
        twist.twist.linear.y = cartForce(1);
        twist.twist.linear.z = cartForce(2);

        twist.twist.angular.x = cartForce(3);
        twist.twist.angular.y = cartForce(4);
        twist.twist.angular.z = cartForce(5);

        pose.pose.position.x = end_frame.p.x();
        pose.pose.position.y = end_frame.p.y();
        pose.pose.position.z = end_frame.p.z();

        end_frame.M.GetQuaternion(pose.pose.orientation.x, pose.pose.orientation.y, pose.pose.orientation.z, pose.pose.orientation.w);
        pose.header.stamp = ros::Time::now();
        twist.header.stamp = ros::Time::now();
        pub.publish(twist);
        posepub.publish(pose);

        fprintf(outfile, " %ld, %2.10f, %2.10f, %2.10f, %2.10f, %2.10f, %2.10f, %2.10f, %2.10f, %2.10f, %2.10f, %2.10f, %2.10f, %2.10f\n",
                timeSec[index], twist.twist.linear.x, twist.twist.linear.y, twist.twist.linear.z,
                twist.twist.angular.x, twist.twist.angular.y, twist.twist.angular.z,
                end_frame.p.x(), end_frame.p.y(), end_frame.p.z(),
                pose.pose.orientation.x, pose.pose.orientation.y, pose.pose.orientation.z, pose.pose.orientation.w);
//        wfile << twist.twist.linear.x << ", " << twist.twist.linear.y << ", " << twist.twist.linear.z << ", "
//              << twist.twist.angular.x << ", " << twist.twist.angular.y << ", " << twist.twist.angular.z << ", "
//              << pose.pose.position.x << ", " << pose.pose.position.y << ", " << pose.pose.position.z << ", "
//              << pose.pose.orientation.x << ", " << pose.pose.orientation.y << ", " << pose.pose.orientation.z << ", " << pose.pose.orientation.w << ", "
//              << offlineData[index][13] << "\n";

        index++;
        if(index > sampleNum - 2)
            ros::shutdown();

        loop_rate.sleep();
    }
    fclose(outfile);
//    wfile.close();
    return 0;
}