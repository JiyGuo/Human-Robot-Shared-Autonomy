//
// Created by root on 8/22/20.
//

#include "robotController/robotController.h"
int main(int argc, char** argv){

    ros::init(argc, argv, "Test");
    ros::NodeHandle *nh;
    nh = new ros::NodeHandle();
    robotController myController(nh);

    ros::AsyncSpinner spinner(2);
    spinner.start();
    ros::Duration(3).sleep();

    KDL::JntArray joint;
    joint.resize(6);
    myController.printCurJoint();
    myController.printCurFrame();

}
