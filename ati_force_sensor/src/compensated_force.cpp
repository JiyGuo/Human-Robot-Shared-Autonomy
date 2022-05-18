//
// Created by zhaoxin on 18-4-26.
//


#include <ros/ros.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/Pose.h>
#include <std_msgs/String.h>
#include <dual_arm_robot.hpp>
#include <actionlib/client/simple_action_client.h>
#include <geometry_msgs/Point.h>
#include "myATI.h"

using namespace KDL;
using namespace Eigen;

JntArray left_curTra_Joint(6);
bool init_flag;

void left_jointstates_sub_callback(const sensor_msgs::JointState &js)
{
    for(int i = 0; i < 6; i++)
    {
        left_curTra_Joint.data[i] = js.position[i];
        init_flag = true;
    }
}

int main(int argc, char **argv) {

    ForceSensor ati_force_sensor;     // 定义一个力传感器
    ati_force_sensor.ATIsensor.initATI(); // 初始化此力传感器
    ati_force_sensor.ATIsensor.biasmeasure();
    ati_force_sensor.grav_compsen.initCalParam();

    ros::init(argc, argv, "calibre_gravity");
    ros::NodeHandle n;
    ros::Subscriber left_jointstates_sub = n.subscribe("left_joint_states", 10, left_jointstates_sub_callback);

    ros::Publisher compen_force_pub = n.advertise<geometry_msgs::TwistStamped>("compensated_force", 10);

    // 通信频率
    ros::Rate loop_rate(125);
    ros::AsyncSpinner spinner(1);
    spinner.start();

    Robot_Kinematics_Annalytical rob_kin;

    init_flag = false;

    while(!init_flag)
    {
        loop_rate.sleep();
    }


    Frame frame_current;
    Rotation fixRot;
    KDL::Wrench ft_current;

    geometry_msgs::TwistStamped twist;


    while(ros::ok())
    {
        rob_kin.FK_left(left_curTra_Joint, frame_current);
        fixRot = Rotation::RotZ(-PI / 4.0);
        frame_current.M = frame_current.M * fixRot;     // 计算当前末端位置
        frame_current.p = frame_current.p + (0.050 + 0.015) * frame_current.M.UnitZ();  // 计算TCP点
        ft_current = ati_force_sensor.getCompsenCartForce(frame_current);  // 测量末端受到的力

        twist.header.stamp = ros::Time::now();
        twist.twist.linear.x = ft_current.force.x();
        twist.twist.linear.y = ft_current.force.y();
        twist.twist.linear.z = ft_current.force.z();

        twist.twist.angular.x = ft_current.torque.x();
        twist.twist.angular.y = ft_current.torque.y();
        twist.twist.angular.z = ft_current.torque.z();

        compen_force_pub.publish(twist);
        loop_rate.sleep();
    }


    return 1;
}
