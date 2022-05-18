//
// Created by zhaoxin on 18-4-20.
//
#include "myATI.h"
#include "sensor_msgs/JointState.h"
#include <ros/ros.h>
#include <kdl/frames.hpp>
#include <kdl/jntarray.hpp>
#include "dual_arm_robot.hpp"
#include "geometry_msgs/TwistStamped.h"

using namespace KDL;
using namespace Eigen;

KDL::JntArray jnt_left;

void readCurrentJntSUB(const sensor_msgs::JointState &JS)
{
    for(int i = 0; i < 6; i++)
        jnt_left.data[i] = JS.position[i];
}

int main(int argc, char **argv)
{
    ForceSensor ati_force_sensor;
    ati_force_sensor.ATIsensor.initATI();
    ati_force_sensor.grav_compsen.initCalParam_dow_init();
    ati_force_sensor.ATIsensor.biasmeasure();
    float f[6];

    jnt_left = JntArray(6);



    ros::init (argc, argv, "calAverageForces");
    ros::NodeHandle node_handle;
    ros::AsyncSpinner spinner(1);
    spinner.start();
    ros::Rate loop_rate(125);

    std::string urdf_param;
    double timeout;
    node_handle.param("timeout", timeout, 0.005);
    node_handle.param("urdf_param", urdf_param, std::string("/robot_description"));
    Robot_KIN rob_kin(urdf_param, "uu_support", "left_tool0", "uu_support", "right_tool0", 0.008);


    ros::Subscriber jntSub = node_handle.subscribe("left_joint_states", 10, readCurrentJntSUB);

    ros::Publisher pub = node_handle.advertise<geometry_msgs::TwistStamped>("/CompenForce", 10);

    Rotation fixRot = Rotation::RotZ(-PI / 4.0);
    Rotation forceSensorRot = Rotation();
    Frame frame_tool0 = Frame();

    KDL::Wrench comedForce;
    geometry_msgs::TwistStamped twist;

    while(ros::ok())
    {
        rob_kin.FK_left(jnt_left, frame_tool0);
        forceSensorRot = frame_tool0.M * fixRot;

        comedForce = ati_force_sensor.getCompsenForce(forceSensorRot);

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
