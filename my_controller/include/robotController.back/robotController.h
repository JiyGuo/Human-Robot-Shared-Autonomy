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

typedef actionlib::SimpleActionClient <control_msgs::FollowJointTrajectoryAction> Client;


class robotController: public Robot_Kinematics_Annalytical {
private:
    //=== ros ===//
    ros::NodeHandle* nh;
    ros::Rate* loop_rate;
    ros::AsyncSpinner* spinner;
    // robot & gripper
//    ros::Publisher gripper_cmd_pub;
    ros::Subscriber joint_state_sub;
    Client *arm_client;

    // variable of robot & gripper control
    std::vector<std::string> jointName;
    control_msgs::GripperCommandActionGoal gripper_cmd;
    bool arm_state;
    control_msgs::FollowJointTrajectoryGoal arm_cmd;
    trajectory_msgs::JointTrajectoryPoint waypoint;
    void joint_statesCB(const sensor_msgs::JointState::ConstPtr &msgs);
    KDL::JntArray joint_state;

    // kinect & realSense

    // variable for reading kinect & realSense

public:
    robotController(ros::NodeHandle *nh);
    ~robotController();

    //robot home;
    KDL::JntArray homeJoint;

    // robot control
    void moveToFrame(const KDL::JntArray jnt_init,  const KDL::Frame& targetFrame,double time, bool wait_for_D);
    void moveToJoint(const KDL::JntArray jnt_target, const double time, bool wait_for_D);
    void getRobotJoints(KDL::JntArray& currJnt);
    void getRobotFrame(KDL::Frame &currFrame);
    void MovePath(std::vector<KDL::Frame> KeyFrames);

    // gripper control
    void gripperAction(const double position, const double effort, double time);

};


#endif //SRC_ROBOTCONTROLLER_H
