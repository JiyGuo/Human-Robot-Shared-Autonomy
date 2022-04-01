//
// Created by root on 8/22/20.
//

#include "robotController/robotController.h"

robotController::robotController(ros::NodeHandle *n) {

    nh = new ros::NodeHandle();
    nh = n;
    loop_rate = new ros::Rate(125);
//    arm_client = new Client("arm_controller/follow_joint_trajectory", true);
    arm_client = new Client("scaled_pos_joint_traj_controller/follow_joint_trajectory", true);

    joint_state_sub = nh->subscribe<sensor_msgs::JointState>("/joint_states", 10, &robotController::joint_statesCB, this);

    jointName.clear();
    jointName.push_back("shoulder_pan_joint");
    jointName.push_back("shoulder_lift_joint");
    jointName.push_back("elbow_joint");
    jointName.push_back("wrist_1_joint");
    jointName.push_back("wrist_2_joint");
    jointName.push_back("wrist_3_joint");

    arm_state = arm_client->waitForServer(ros::Duration(3));
    if(arm_state) ROS_INFO("connected the arm server.");
    else {ROS_WARN("connect the arm server failed.");}
    waypoint.positions.clear();
    waypoint.positions.resize(6);
    arm_cmd.trajectory.joint_names.resize(6);
    arm_cmd.trajectory.joint_names = jointName;
    joint_state.resize(6);
    joint_state.resize(6);
    homeJoint.resize(6);
    homeJoint.data<<0,-1.95667,1.84611,-1.46024,-1.5708,-0.0865585 ;

    spinner = new ros::AsyncSpinner(2);
    spinner->start();
    ros::Duration(4.0).sleep();
}

void robotController::gripperAction(const double pos, const double effort, double time){
    gripper_cmd.goal.command.position = pos;
    gripper_cmd.goal.command.max_effort = effort;
//    gripper_cmd_pub.publish(gripper_cmd);
    ros::Duration(time).sleep();
}

void robotController::moveToJoint(const KDL::JntArray jnt_target, const double time, bool wait_for_D) {

    waypoint.positions.clear();
    for (int i = 0; i < 6; ++i) {
        waypoint.positions.push_back(jnt_target.data[i]);
    }
    waypoint.time_from_start = ros::Duration(time);
    arm_cmd.trajectory.points.clear();
    arm_cmd.trajectory.points.push_back(waypoint);
    arm_cmd.trajectory.header.stamp = ros::Time::now();
    arm_client->sendGoal(arm_cmd);
    if (wait_for_D){
        ros::Duration(time).sleep();
    }
}

void robotController::getRobotJoints(KDL::JntArray &currJnt) {
    currJnt = joint_state;
}

void robotController::getRobotFrame(KDL::Frame &currFrame) {
    KDL::JntArray currJnt;
    currJnt.resize(6);
    getRobotJoints(currJnt);
//    std::cout<<"RobotJoints:"<<"\t";
//    for (int i = 0; i < currJnt.data.size(); ++i) {
//        std::cout<<currJnt.data[i]<<"\t";
//    }
//    std::cout<<std::endl;
    FKine(currJnt,currFrame);
}


void robotController::joint_statesCB(const sensor_msgs::JointState::ConstPtr &msgs) {
    for (int i = 0; i < msgs->name.size(); ++i) {
        if (msgs->name.at(i) == jointName.at(0)){
            joint_state.data[0] = msgs->position.at(i);
        } else if (msgs->name.at(i) == jointName.at(1)){
            joint_state.data[1] = msgs->position.at(i);
        } else if (msgs->name.at(i) == jointName.at(2)){
            joint_state.data[2] = msgs->position.at(i);
        } else if (msgs->name.at(i) == jointName.at(3)){
            joint_state.data[3] = msgs->position.at(i);
        } else if (msgs->name.at(i) == jointName.at(4)){
            joint_state.data[4] = msgs->position.at(i);
        } else if (msgs->name.at(i) == jointName.at(5)){
            joint_state.data[5] = msgs->position.at(i);
        }
    }
//    std::cout<<"RobotJoints:"<<"\t";
//    for (int i = 0; i < 6; ++i) {
//        std::cout<<joint_state.data[i]<<"\t";
//    }
//    std::cout<<std::endl;
}

void robotController::moveToFrame(const KDL::JntArray jnt_init,  const KDL::Frame& targetFrame,double time, bool wait_for_D){
    KDL::JntArray jnt_target;
    jnt_target.resize(6);
    IK_analytical(jnt_init,targetFrame,jnt_target);
    moveToJoint( jnt_target, time,  wait_for_D);
}

void robotController::MovePath(std::vector<KDL::Frame> KeyFrames){
    KDL::JntArray jnt_target, joint_curr;
    joint_curr.resize(6);
    jnt_target.resize(6);
    double time_step = 0.008;
    bool wait_for_D = true;
    KDL::Frame currFrame;
    getRobotFrame(currFrame);
    try {
        KDL::Path_RoundedComposite* path = new KDL::Path_RoundedComposite(0.005,0.01,new KDL::RotationalInterpolation_SingleAxis());
        path->Add(currFrame);
        for (int i = 0; i < KeyFrames.size(); ++i) {
            path->Add(KeyFrames[i]);
        }
        path->Finish();
        KDL::VelocityProfile* velpref = new KDL::VelocityProfile_Trap(0.05,0.1);
        velpref->SetProfile(0,path->PathLength());
        KDL::Trajectory* traject = new KDL::Trajectory_Segment(path, velpref);
        double time = 0;
        while (time <= traject->Duration()) {
            getRobotJoints(joint_curr);
            moveToFrame(joint_curr,  traject->Pos(time) , time_step, wait_for_D);
            time += time_step;
        }
//        delete path;
//        delete velpref;
        delete traject;
//        return traject;
    } catch(KDL::Error& error) {
        std::cout <<"I encountered this error : " << error.Description() << std::endl;
        std::cout <<"with the following type " << error.GetType() << std::endl;
    }
}

robotController::~robotController() {
    spinner->stop();
    delete spinner;
    delete loop_rate;
    delete arm_client;
    delete nh;
}