//
// Created by zhaoxin on 17-10-28.
//

#include "myATI.h"
#include "ros/ros.h"
#include <kdl/frames.hpp>
#include <geometry_msgs/TwistStamped.h>
#include <Eigen/Core>
#include <jntarray.hpp>
#include <sensor_msgs/JointState.h>
#include "dual_arm_robot.hpp"
//#include <eigen_conversions/eigen_kdl.h>

KDL::Wrench measureForce;
KDL::JntArray jntLeft;

using namespace KDL;
void readCurrentJntSUB(const sensor_msgs::JointState &JS)
{
    for(int i = 0; i < 6; i++)
        jntLeft.data[i] = JS.position[i];
}

void readCurrentForceSensorSUB(const geometry_msgs::TwistStampedPtr &msg)
{
    measureForce.force.data[0] = msg->twist.linear.x;
    measureForce.force.data[1] = msg->twist.linear.y;
    measureForce.force.data[2] = msg->twist.linear.z;
    measureForce.torque.data[0] = msg->twist.angular.x;
    measureForce.torque.data[1] = msg->twist.angular.y;
    measureForce.torque.data[2] = msg->twist.angular.z;
}

int main(int argc, char **argv)
{
    jntLeft = JntArray(6);

    ATIGravityCompensation gravityCom;
    gravityCom.initCalParam();

    ros::init (argc, argv, "calAverageForces");
    ros::NodeHandle node_handle;
    ros::AsyncSpinner spinner(1);
    spinner.start();
    ros::Rate loop_rate(50);

    std::string urdf_param;
    double timeout;
    node_handle.param("timeout", timeout, 0.005);
    node_handle.param("urdf_param", urdf_param, std::string("/robot_description"));
    Robot_KIN rob_kin(urdf_param, "uu_support", "left_tool0", "uu_support", "right_tool0", 0.008);

    // 建立两个订阅器，保存实时的位置和力信息
    ros::Subscriber forceSub = node_handle.subscribe("ATI_force", 10, readCurrentForceSensorSUB);
    ros::Subscriber jntSub = node_handle.subscribe("left_joint_states", 10, readCurrentJntSUB);

    ros::Publisher pub = node_handle.advertise<geometry_msgs::TwistStamped>("/CompenForce", 10);

//    Rotation robR = Rotation();
    Rotation fixRot = Rotation::RotZ(-PI / 4.0);
    Rotation forceSensorRot = Rotation();
    Frame frame_tool0 = Frame();

    KDL::Wrench comedForce;
    geometry_msgs::TwistStamped twist;

    Eigen::Matrix<double, 6, 1> cartForce;

    while(ros::ok())
    {
        rob_kin.FK_left(jntLeft, frame_tool0);
        forceSensorRot = frame_tool0.M * fixRot;
//        comedForce = gravityCom.compensete(measureForce, forceSensorRot);
        cartForce = gravityCom.getCartWrench(measureForce, forceSensorRot, frame_tool0.p  );
        comedForce.force.data[0] = cartForce(0);
        comedForce.force.data[1] = cartForce(1);
        comedForce.force.data[2] = cartForce(2);
        comedForce.torque.data[0] = cartForce(3);
        comedForce.torque.data[1] = cartForce(4);
        comedForce.torque.data[2] = cartForce(5);

//        tf::wrenchEigenToKDL(cartForce, comedForce);
//        std::cout << "tool0 pos:" << frame_tool0.p[0] << "  " << frame_tool0.p[1] << "  " << frame_tool0.p[2] << std::endl;
//
//        std::cout << "Rotation: " <<std::endl ;
//        for(int i = 0; i < 3; i++)
//        {
//            for(int j = 0; j < 3; j++)
//                std::cout << forceSensorRot.data[i * 3 + j] << "  ";
//            std::cout << std::endl;
//        }


//        std::cout << " x : " << comedForce.force.x() << ", ";
//        std::cout << " Y : " << comedForce.force.y() << ", ";
//        std::cout << " x : " << comedForce.force.z() << ", ";
//
//        std::cout << " X : " << comedForce.torque.x() << ", ";
//        std::cout << " Y : " << comedForce.torque.y() << ", ";
//        std::cout << " Z : " << comedForce.torque.z() << std::endl;

        twist.twist.linear.x = comedForce.force.x();
        twist.twist.linear.y = comedForce.force.y();
        twist.twist.linear.z = comedForce.force.z();

        twist.twist.angular.x = comedForce.torque.x();
        twist.twist.angular.y = comedForce.torque.y();
        twist.twist.angular.z = comedForce.torque.z();

        twist.header.stamp = ros::Time::now();
        pub.publish(twist);

        loop_rate.sleep();
    }

    return 1;
}