//
// Created by jiyguo on 20-10-25.
//
#include <vector>
#include <string>
#include "robotController/robotController.h"

using namespace std;
int main(int argc, char** argv){

    ros::init(argc, argv, "Test");
    ros::NodeHandle *nh;
    nh = new ros::NodeHandle();
    robotController myController(nh);

    ros::AsyncSpinner spinner(2);
    spinner.start();
    ros::Duration(3).sleep();

//    myController.moveToJoint(myController.homeJoint,5);
    ros::Duration(1).sleep();

    KDL::Frame initFrame;
    KDL::Frame targetFrame;

    initFrame.M = KDL::Rotation::RotX(M_PI);
    initFrame.p = KDL::Vector(0.21723, 0.25364, 0.1);
    KDL::JntArray jnt;
    myController.getRobotJoints(jnt);
    myController.printCurJoint();
    myController.printCurFrame();

    myController.moveToFrame(jnt,initFrame,10, true);

}


