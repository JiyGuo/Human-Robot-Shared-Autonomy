//
// Created by root on 8/22/20.
//
#include "robotController/robotController.h"
robotController:: robotController(ros::NodeHandle *n,bool sim)
{
    nh = n;
    loop_rate = new ros::Rate(frequency);

    if (sim){
        prefix = "";
        ns = "";
        arm_client = new Client("/arm_controller/follow_joint_trajectory", true);
    }else{
        prefix = "right_";
        ns = "ur5/";
        arm_client = new Client("/right_ur5/right_pos_based_controller/follow_joint_trajectory", true);
    }
    gripper_cmd_pub = nh->advertise<control_msgs::GripperCommandActionGoal>("gripper_controller/gripper_cmd/goal", 10);
    joint_state_sub = nh->subscribe<sensor_msgs::JointState>("/"+prefix+ns+prefix+"joint_states", 10, &robotController::joint_statesCB, this);
    jointName.clear();
    jointName.emplace_back(prefix+"shoulder_pan_joint");
    jointName.emplace_back(prefix+"shoulder_lift_joint");
    jointName.emplace_back(prefix+"elbow_joint");
    jointName.emplace_back(prefix+"wrist_1_joint");
    jointName.emplace_back(prefix+"wrist_2_joint");
    jointName.emplace_back(prefix+"wrist_3_joint");
    arm_state = arm_client->waitForServer(ros::Duration(3));
    if(arm_state) ROS_INFO("connected the arm server.");
    else {ROS_WARN("connect the arm server failed.");}
    waypoint.positions.resize(6);
    waypoint.velocities.resize(6);
    waypoint.accelerations.resize(6);
    arm_cmd.trajectory.joint_names.resize(6);
    arm_cmd.trajectory.joint_names = jointName;
    joint_state.resize(6);
    joint_speed.resize(6);
    homeJoint.resize(6);
    refferJoint.resize(6);
    homeJoint.data << 0, -1.13, 1.73, -2.05, 2.15, -0.47;
    refferJoint.data << 0,-1.325,1.535,-1.81,-1.64,0;
}

void robotController::gripperAction(const double& pos, const double& effort,const double& time){
    gripper_cmd.goal.command.position = pos;
    gripper_cmd.goal.command.max_effort = effort;
    gripper_cmd_pub.publish(gripper_cmd);
    ros::Duration(time).sleep();
}

void robotController::moveToJoint(const KDL::JntArray& jnt_target, const double& time, bool wait_for_D) {
    KDL::JntArray jnt_speed(6); jnt_speed.data.setZero();
    moveToJoint(jnt_target,jnt_speed,time,wait_for_D);
}

void robotController::moveToJoint(const KDL::JntArray& jnt_target,const KDL::JntArray& jnt_speed, const double& time, bool wait_for_D) {
    trajectory_msgs::JointTrajectoryPoint p1;
    control_msgs::FollowJointTrajectoryGoal g;
    g.trajectory.header.stamp = ros::Time::now();
    for(int i = 0; i < 6; i++)
    {
        g.trajectory.joint_names.push_back(jointName[i]);
        p1.positions.push_back(jnt_target.data[i]);
        p1.velocities.push_back(0);
    }
    p1.time_from_start = ros::Duration(time);
    g.trajectory.points.push_back(p1);
    arm_client->sendGoal(g);
    if(wait_for_D)
        ros::Duration(time).sleep();

    //    waypoint.positions.clear();
//    waypoint.velocities.clear();
//    waypoint.accelerations.clear();
//    arm_cmd.trajectory.points.clear();
//    arm_cmd.trajectory.header.stamp = ros::Time::now();
//    for (int i = 0; i < jnt_target.data.size(); ++i) {
//        waypoint.positions[i] = jnt_target.data[i];
//        waypoint.velocities[i] = jnt_speed.data[i];
//        waypoint.accelerations[i] = 0;
//    }
//    waypoint.time_from_start = ros::Duration(time);
//    arm_cmd.trajectory.points.push_back(waypoint);
//    arm_client->sendGoal(arm_cmd);
//    if (wait_for_D){
//        ros::Duration(time).sleep();
//    }
}

void robotController::getRobotJoints(KDL::JntArray &currJnt) {
    currJnt = this->joint_state;
}
void robotController::getRobotJointSpeed(KDL::JntArray& jntSpeed){
    jntSpeed = this->joint_speed;
}

void robotController::getRobotVelocity(KDL::Twist &Vee) {
    KDL::Jacobian jac(6);
//    std::cout<<1<<std::endl;
    GeometryJacobian::JntToJacobianRight(this->joint_state,jac);
    KDL::JntArray speed(6);
    speed.data = jac.data * this->joint_speed.data;
    Vee.vel.data[0] = speed.data(0);
    Vee.vel.data[1] = speed.data(1);
    Vee.vel.data[2] = speed.data(2);
    Vee.rot.data[0] = speed.data(3);
    Vee.rot.data[1] = speed.data(4);
    Vee.rot.data[2] = speed.data(5);
}

void robotController::getRobotFrame(KDL::Frame &currFrame) {
    kinematics.FKine(joint_state,currFrame);
}

void robotController::printCurJoint() {
    std::cout << "RobotJoints:" << "\t";
    for (int i = 0; i < joint_state.data.size(); ++i) {
        std::cout << joint_state.data[i] << "\t";
    }
    std::cout << std::endl;
}

void robotController::printCurFrame() {
    KDL::Frame curFrame;
    getRobotFrame(curFrame);
    double frame[4][4];
    curFrame.Make4x4(*frame);
    std::cout << "RobotFrame:" << "\n";
    for (int i = 0; i < 4; ++i) {
        for (int j = 0; j < 4; ++j) {
            std::cout <<frame[i][j]<< "\t";
        }
        std::cout << std::endl;
    }
}

void robotController::printFrame(KDL::Frame Frame) {
    double frame[4][4];
    Frame.Make4x4(*frame);
    for(int i = 0; i < 4; ++i) {
        for (int j = 0; j < 4; ++j) {
            std::cout <<frame[i][j]<< "\t";
        }
        std::cout << std::endl;
    }
}

void robotController::joint_statesCB(const sensor_msgs::JointState::ConstPtr &msgs) {
    for (int i = 0; i < msgs->name.size(); ++i) {
        if (msgs->name.at(i) == jointName.at(0)){
            joint_state.data[0] = msgs->position.at(i);
            joint_speed.data[0] = msgs->velocity.at(i);
        } else if (msgs->name.at(i) == jointName.at(1)){
            joint_state.data[1] = msgs->position.at(i);
            joint_speed.data[1] = msgs->velocity.at(i);
        } else if (msgs->name.at(i) == jointName.at(2)){
            joint_state.data[2] = msgs->position.at(i);
            joint_speed.data[2] = msgs->velocity.at(i);
        } else if (msgs->name.at(i) == jointName.at(3)){
            joint_state.data[3] = msgs->position.at(i);
            joint_speed.data[3] = msgs->velocity.at(i);
        } else if (msgs->name.at(i) == jointName.at(4)){
            joint_state.data[4] = msgs->position.at(i);
            joint_speed.data[4] = msgs->velocity.at(i);
        } else if (msgs->name.at(i) == jointName.at(5)){
            joint_state.data[5] = msgs->position.at(i);
            joint_speed.data[5] = msgs->velocity.at(i);
        }
    }
}

void robotController::moveToFrame(const KDL::JntArray& jnt_init,  const KDL::Frame& targetFrame,const double& time, bool wait_for_D){
    KDL::JntArray jnt_target;
    jnt_target.resize(6);
    kinematics.IK_analytical(jnt_init,targetFrame,jnt_target);
    moveToJoint( jnt_target, time,  wait_for_D);
}

void robotController::moveToFrame(const KDL::Frame& targetFrame,const double& time, bool wait_for_D){
    KDL::JntArray jnt_target;
    jnt_target.resize(6);
    kinematics.IK_analytical(joint_state, targetFrame,jnt_target);
    moveToJoint( jnt_target, time,  wait_for_D);
}

void robotController::moveToFrame(const KDL::Frame& targetFrame, const KDL::JntArray& jnt_speed, const double& time, bool wait_for_D){
    KDL::JntArray jnt_target;
    jnt_target.resize(6);
    kinematics.IK_analytical(joint_state, targetFrame,jnt_target);
    moveToJoint( jnt_target,jnt_speed, time,  wait_for_D);
}

//void robotController::moveToCameraFrame(const KDL::Frame& targetFrame,double time, bool wait_for_D){
//    KDL::JntArray jnt_target;
//    jnt_target.resize(6);
//    KDL::Frame T_c2ee , toolTarget;
//    if (realsense_link == "/realsense_camera_link")
//        toolTarget = targetFrame * KDL::Frame(KDL::Rotation::RPY(-M_PI_2,0,-M_PI_2),KDL::Vector(0,0,0)).Inverse();
//    else
//        toolTarget = targetFrame;
//    getTfTranslation(realsense_link,"/ee_link",  T_c2ee);
//    toolTarget = toolTarget * T_c2ee * toolFrame;
//    moveToFrame(joint_state, toolTarget,time, wait_for_D);
//}

void robotController::MovePath(std::vector<KDL::Frame> KeyFrames){
    KDL::JntArray jnt_target, joint_curr;
    joint_curr.resize(6);
    jnt_target.resize(6);
    double time_step = 1.0/frequency;
    bool wait_for_D = true;
    KDL::Frame currFrame;
    sleep(1);
    getRobotFrame(currFrame);
    try {
        KDL::Path_RoundedComposite* path = new KDL::Path_RoundedComposite(0.01,0.01,new KDL::RotationalInterpolation_SingleAxis());
        path->Add(currFrame);
        for (int i = 0; i < KeyFrames.size(); ++i) {
            path->Add(KeyFrames[i]);
        }
        path->Finish();
        KDL::VelocityProfile* velpref = new KDL::VelocityProfile_Trap(0.1,0.25);
        velpref->SetProfile(0,path->PathLength());
        KDL::Trajectory* traject = new KDL::Trajectory_Segment(path, velpref);
        double time = 0;
        while (time <= traject->Duration()) {
            moveToFrame(joint_state,  traject->Pos(time) , time_step, wait_for_D);
            time += time_step;
        }
        ros::Duration(0.5).sleep();
//        delete path;
//        delete velpref;
//        delete traject;
//        return traject;
    } catch(KDL::Error& error) {
        std::cout <<"I encountered this error : " << error.Description() << std::endl;
        std::cout <<"with the following type " << error.GetType() << std::endl;
    }
}

//void robotController::MoveCameraPath(std::vector<KDL::Frame> KeyFrames){
//    KDL::JntArray jnt_target, joint_curr;
//    joint_curr.resize(6);
//    jnt_target.resize(6);
//    double time_step = 1.0/frequency;
//    bool wait_for_D = true;
//    KDL::Frame currFrame;
//    sleep(1);
//    getRobotFrame(currFrame);
//    try {
//        KDL::Path_RoundedComposite* path = new KDL::Path_RoundedComposite(0.01,0.01,new KDL::RotationalInterpolation_SingleAxis());
//        path->Add(currFrame);
//        for (int i = 0; i < KeyFrames.size(); ++i) {
//            path->Add(KeyFrames[i]);
//        }
//        path->Finish();
//        KDL::VelocityProfile* velpref = new KDL::VelocityProfile_Trap(0.25,0.5);
//        velpref->SetProfile(0,path->PathLength());
//        KDL::Trajectory* traject = new KDL::Trajectory_Segment(path, velpref);
//        double time = 0;
//        while (time <= traject->Duration()) {
//            moveToCameraFrame(traject->Pos(time) , time_step,true);
//            time += time_step;
//        }
//        ros::Duration(0.5).sleep();
////        delete path;
////        delete velpref;
////        delete traject;
//    } catch(KDL::Error& error) {
//        std::cout <<"I encountered this error : " << error.Description() << std::endl;
//        std::cout <<"with the following type " << error.GetType() << std::endl;
//    }
//}

KDL::Trajectory* robotController::getCameraPath(std::vector<KDL::Frame> KeyFrames) {
    KDL::JntArray jnt_target, joint_curr;
    joint_curr.resize(6);
    jnt_target.resize(6);
    KDL::Frame currFrame;
    sleep(1);
    getRealSense_camera_link(currFrame);
    if (realsense_link == "/realsense_camera_link")
        currFrame = currFrame * KDL::Frame(KDL::Rotation::RPY(-M_PI_2,0,-M_PI_2),KDL::Vector(0,0,0));
    try {
        KDL::Path_RoundedComposite* path = new KDL::Path_RoundedComposite(0.01,0.01,new KDL::RotationalInterpolation_SingleAxis());
        path->Add(currFrame);
        for (int i = 0; i < KeyFrames.size(); ++i) {
            path->Add(KeyFrames[i]);
        }
        path->Finish();
        KDL::VelocityProfile* velpref = new KDL::VelocityProfile_Trap(0.1,0.2);
        velpref->SetProfile(0,path->PathLength());
        KDL::Trajectory* traject = new KDL::Trajectory_Segment(path, velpref);
//        std::cout<<traject->Duration()<<std::endl;
        return traject;
    } catch(KDL::Error& error) {
        std::cout <<"I encountered this error : " << error.Description() << std::endl;
        std::cout <<"with the following type " << error.GetType() << std::endl;
    }
}


KDL::Trajectory* robotController::getPath(std::vector<KDL::Frame> KeyFrames){
    KDL::JntArray jnt_target, joint_curr;
    joint_curr.resize(6);
    jnt_target.resize(6);
    KDL::Frame currFrame;
    sleep(1);
    getRobotFrame(currFrame);
    try {
        KDL::Path_RoundedComposite* path = new KDL::Path_RoundedComposite(0.01,0.01,new KDL::RotationalInterpolation_SingleAxis());
        path->Add(currFrame);
        for (int i = 0; i < KeyFrames.size(); ++i) {
            path->Add(KeyFrames[i]);
        }
        path->Finish();
        KDL::VelocityProfile* velpref = new KDL::VelocityProfile_Trap(0.1,0.2);
        velpref->SetProfile(0,path->PathLength());
        KDL::Trajectory* traject = new KDL::Trajectory_Segment(path, velpref);
//        std::cout<<traject->Duration()<<std::endl;
        return traject;
    } catch(KDL::Error& error) {
        std::cout <<"I encountered this error : " << error.Description() << std::endl;
        std::cout <<"with the following type " << error.GetType() << std::endl;
    }
}

void robotController::grasp(const KDL::Frame& initFrame , const KDL::Frame& targetFrame  , double ext_time , double height){
    double position_gripper = 0;
    double gripper_time = 1;
    double effort_gripper = 30.0;
    KDL::Frame tmpFrame;
    /* Move up to the top of object */
    tmpFrame.M = initFrame.M;
    tmpFrame.p = initFrame.p + KDL::Vector(0,0,height);
    moveToFrame(tmpFrame,5);
    /* Move down to grisp object */
    std::vector<KDL::Frame> objFrames;
    objFrames.clear();
    objFrames.push_back(initFrame);
    MovePath(objFrames);

    /* Grisp object */
    /*
    position_gripper = 0.085;
    gripperAction(position_gripper, effort_gripper, gripper_time);
     */

    /* Lift and move to target position*/
    objFrames.clear();
    tmpFrame.M = initFrame.M;
    tmpFrame.p = initFrame.p + KDL::Vector(0,0,height);

    objFrames.push_back(tmpFrame);
    MovePath(objFrames);

    tmpFrame.p = targetFrame.p + KDL::Vector(0,0,height);
    tmpFrame.M = targetFrame.M;
    moveToFrame(refferJoint,tmpFrame,ext_time);

    objFrames.clear();
    objFrames.push_back(targetFrame);
    MovePath(objFrames);
    /*
    position_gripper = 0;
    gripperAction(position_gripper, effort_gripper, gripper_time);
    */

    objFrames.clear();
    tmpFrame.p = targetFrame.p + KDL::Vector(0,0,height);
    objFrames.push_back(tmpFrame);
    MovePath(objFrames);
}

void robotController::getRealSense_camera_link(KDL::Frame &Frame){
    tf::TransformListener tf;
    tf::StampedTransform tf_transform;
    tf.waitForTransform("/world",realsense_link,ros::Time(),ros::Duration(0.5));
    tf.lookupTransform("/world",realsense_link,ros::Time(),tf_transform);
    double yaw, pitch, roll;
    tf_transform.getBasis().getRPY(roll, pitch, yaw);
    tf::Vector3 v = tf_transform.getOrigin();
    Frame.M = KDL::Rotation::RPY(roll, pitch,yaw);
    Frame.p = KDL::Vector(v.x(),v.y(),v.z());
}

void robotController::getKinect_camera_link(KDL::Frame &Frame){
    tf::TransformListener tf;
    tf::StampedTransform tf_transform;
    tf.waitForTransform("/world",kinect_link,ros::Time(),ros::Duration(0.5));
    tf.lookupTransform("/world",kinect_link,ros::Time(),tf_transform);
    double yaw, pitch, roll;
    tf_transform.getBasis().getRPY(roll, pitch, yaw);
    tf::Vector3 v = tf_transform.getOrigin();
    Frame.M = KDL::Rotation::RPY(roll, pitch,yaw);
    Frame.p = KDL::Vector(v.x(),v.y(),v.z());
}

void robotController::getTfTranslation(const std::string& base_frameid,const std::string& target_frameid,  KDL::Frame &Frame){
    tf::TransformListener tfListener;
    tf::StampedTransform tf_transform;
    try {
        tfListener.waitForTransform(base_frameid, target_frameid, ros::Time(), ros::Duration(0.5));
        tfListener.lookupTransform(base_frameid, target_frameid, ros::Time(), tf_transform);
        double yaw, pitch, roll;
        tf_transform.getBasis().getRPY(roll, pitch, yaw);
        tf::Vector3 v = tf_transform.getOrigin();
        Frame.M = KDL::Rotation::RPY(roll, pitch, yaw);
        Frame.p = KDL::Vector(v.x(), v.y(), v.z());
    }
    catch(tf::TransformException& ex)
    {
        std::cout << "Failure at "<< ros::Time::now() << std::endl;
        std::cout << "Exception thrown:" << ex.what()<< std::endl;
        std::cout << "The current list of frames is:" <<std::endl;
        std::cout << tfListener.allFramesAsString() <<std::endl;
    }
}

void robotController::FKine(const KDL::JntArray& joint,KDL::Frame &currFrame){
    kinematics.FKine(joint,currFrame);
}

void robotController::IK_analytical(const KDL::JntArray& jnt_init, const KDL::Frame& frame, KDL::JntArray &jnt_out){
    kinematics.IK_analytical(jnt_init, frame, jnt_out);
}

robotController::~robotController() {
//    spinner->stop();
//    delete spinner;
    delete loop_rate;
    loop_rate = nullptr;
    delete arm_client;
    arm_client = nullptr;
//    delete nh;
}
