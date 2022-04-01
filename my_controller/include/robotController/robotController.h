//
// Created by root on 8/22/20.
//

#ifndef SRC_ROBOTCONTROLLER_H
#define SRC_ROBOTCONTROLLER_H

#include "control_msgs/GripperCommandActionGoal.h"
#include "control_msgs/FollowJointTrajectoryActionGoal.h"
#include "control_msgs/FollowJointTrajectoryGoal.h"
#include "control_msgs/FollowJointTrajectoryAction.h"
#include "trajectory_msgs/JointTrajectory.h"
#include "trajectory_msgs/JointTrajectoryPoint.h"
#include "std_msgs/String.h"
#include "sensor_msgs/JointState.h"
#include "actionlib/client/simple_action_client.h"
#include "ros/ros.h"
#include "kdl/jntarray.hpp"
#include "ur3e_kin.hpp"
#include "tf/transform_listener.h"
#include "jacobian.h"


typedef actionlib::SimpleActionClient <control_msgs::FollowJointTrajectoryAction> Client;


class robotController{
private:
    //=== ros ===//
    ros::NodeHandle *nh;
    ros::Rate* loop_rate;

    ros::Publisher gripper_cmd_pub;
    ros::Subscriber joint_state_sub;
    Client *arm_client;

    std::string prefix;
    std::string ns;
    Robot_Kinematics_Annalytical kinematics;

    // variable of robot & gripper control
    std::vector<std::string> jointName;
    control_msgs::GripperCommandActionGoal gripper_cmd;
    bool arm_state;
    bool if_exceed;
    control_msgs::FollowJointTrajectoryGoal arm_cmd;
    trajectory_msgs::JointTrajectoryPoint waypoint;
    void joint_statesCB(const sensor_msgs::JointState::ConstPtr &msgs);
    KDL::JntArray joint_state;
    KDL::JntArray joint_speed;
    const int frequency = 125;

    std::string robot_control;
    std::string realsense_link;
    std::string kinect_link;


public:
    explicit robotController(ros::NodeHandle *n,bool sim = true);
    ~robotController();

    //robot home;
    KDL::JntArray homeJoint;
    KDL::JntArray refferJoint;

    // robot control
    void moveToFrame(const KDL::JntArray& jnt_init,  const KDL::Frame& targetFrame,const double& time, bool wait_for_D = true);
    void moveToFrame(const KDL::Frame& targetFrame,const double& time, bool wait_for_D = true);
    void moveToFrame(const KDL::Frame& targetFrame, const KDL::JntArray& jnt_init, const double& time, bool wait_for_D = true);

//    void moveToCameraFrame(const KDL::Frame& targetFrame,double time, bool wait_for_D = true);

    void moveToJoint(const KDL::JntArray& jnt_target, const double& time, bool wait_for_D = true);
    void moveToJoint(const KDL::JntArray& jnt_target, const KDL::JntArray& jnt_speed, const double& time, bool wait_for_D = true);

    void FKine(const KDL::JntArray& homeJoint,KDL::Frame &currFrame);
    void IK_analytical(const KDL::JntArray& jnt_init, const KDL::Frame& frame, KDL::JntArray &jnt_out);

    void getRobotJoints(KDL::JntArray& currJnt);
    void getRobotJointSpeed(KDL::JntArray& jntSpeed);

    void getRobotFrame(KDL::Frame &currFrame);
    void getRobotVelocity(KDL::Twist &Vee);
    KDL::Trajectory* getPath(std::vector<KDL::Frame> KeyFrames);
    KDL::Trajectory* getCameraPath(std::vector<KDL::Frame> KeyFrames);

    void MovePath(std::vector<KDL::Frame> KeyFrames);
//    void MoveCameraPath(std::vector<KDL::Frame> KeyFrames);

    void printCurJoint();
    void printCurFrame();
    static void printFrame(KDL::Frame);


    // gripper control
    void gripperAction(const double& pos, const double& effort,const double& time);

    // Full grisp
//    void grasp(const KDL::Frame initFrame,const KDL::Vector initVec,const KDL::Frame targetFrame,const KDL::Vector targetVec,double height);
    void grasp(const KDL::Frame& initFrame, const KDL::Frame& targetFrame , double ext_time = 3 , double height = 0.1 );

    void getRealSense_camera_link(KDL::Frame &Frame);
    void getKinect_camera_link(KDL::Frame &Frame);
    void getTfTranslation(const std::string& base_frameid,const std::string& target_frameid,  KDL::Frame &Frame);

};


#endif //SRC_ROBOTCONTROLLER_H
