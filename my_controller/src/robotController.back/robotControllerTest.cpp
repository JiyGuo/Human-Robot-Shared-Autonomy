//
// Created by root on 8/22/20.
//

#include "robotController/robotController.h"
int main(int argc, char** argv){

    ros::init(argc, argv, "Test");
    ros::NodeHandle *nh;
    nh = new ros::NodeHandle();
    robotController myController(nh);
    double position = 1;
    double effort = 0.2;
    double gripper_time = 1;
    KDL::JntArray joint;
    joint.resize(6);

    myController.getRobotJoints(joint);
    std::cout<<"RobotJoints:"<<"\t";
    for (int i = 0; i < joint.data.size(); ++i) {
        std::cout<<joint.data[i]<<"\t";
    }
    std::cout<<std::endl;

    KDL::JntArray home_jnt;
    home_jnt = myController.homeJoint;
    myController.moveToJoint(home_jnt,5,true);

    KDL::Frame targetFrame;
    Eigen::Vector3d targePos(-0.00,0.40,0.2);

    //    targetFrame.M =  KDL::Rotation::RotZ(-M_PI/6) * KDL::Rotation::RotY(M_PI) ;

    std::vector<KDL::Frame> objFrames;
    objFrames.clear();
    targetFrame.M =  KDL::Rotation::RotZ(0) * KDL::Rotation::RotY(M_PI) ;
    targetFrame.p = KDL::Vector(targePos[0],targePos[1],0.2);

//    myController.moveToFrame( joint, targetFrame, 2, true);


    objFrames.push_back(targetFrame);
//    myController.MovePath(objFrames);
//
//    objFrames.clear();
    targetFrame.p.z(0.1) ;
    objFrames.push_back(targetFrame);
    myController.MovePath(objFrames);

    myController.getRobotJoints(joint);
    std::cout<<"RobotJoints:"<<"\t";
    for (int i = 0; i < joint.data.size(); ++i) {
        std::cout<<joint.data[i]<<"\t";
    }
    std::cout<<std::endl;
}
