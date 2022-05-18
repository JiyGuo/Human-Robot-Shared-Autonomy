//
// Created by zhaoxin on 18-4-20.
//
#include "robotController/robotController.h"
#include "myATI.h"
#include "sensor_msgs/JointState.h"
#include <ros/ros.h>
#include <kdl/frames.hpp>
#include <kdl/jntarray.hpp>
#include "geometry_msgs/TwistStamped.h"


using namespace KDL;
using namespace Eigen;

KDL::JntArray jnt_left_initial(6);
KDL::JntArray jnt_left_final(6);
KDL::JntArray jnt_left_delta(6);

std::vector<JntArray> joint_points;

/**
 * 初始化要标定的6个点的位置
 */
void position_init()
{
/*    jnt_left_initial.data<<-0.41866609, -1.33512103,  1.33469517, -1.24456985, -0.85454636,  3.4450705;
    jnt_left_final.data<<-0.22533422, -1.30980328,  1.64924888, -1.756447,   -3.94990699,  4.15445958;*/
    jnt_left_initial.data<<-0.41866609, -1.33512103,  1.33469517, -1.24456985, -0.85454636,  3.4450705;
    jnt_left_final.data<<-0.41866609, -1.33512103,  1.33469517, -1.24456985, -0.85454636,  3.4450705;
    jnt_left_delta.data=jnt_left_final.data-jnt_left_initial.data;

    //差值的路径点
    joint_points.resize(100);

    for(int i = 0; i < 100; i++)
    {
        joint_points[i].resize(6);
        joint_points[i].data.setZero();
    }
}

int main(int argc, char **argv)
{
    position_init();

    ros::init (argc, argv, "calAverageForces");
    ros::NodeHandle *node_handle;
    node_handle = new ros::NodeHandle();
    robotController myController(node_handle,"right",false);
    ros::AsyncSpinner spinner(1);
    spinner.start();
    ros::Rate loop_rate(125);

    std::string urdf_param;
    double timeout;
    node_handle->param("timeout", timeout, 0.005);
    node_handle->param("urdf_param", urdf_param, std::string("/robot_description"));

    //ros::Publisher pub1 = node_handle->advertise<geometry_msgs::TwistStamped>("/ATI_force", 10);
    ros::Publisher pub = node_handle->advertise<geometry_msgs::TwistStamped>("/CompenForce", 10);

    Rotation fixRot = Rotation::RotZ(-PI / 4.0);
    Rotation forceSensorRot = Rotation();
    Frame frame_tool0 = Frame();

    KDL::Wrench comedForce;
    geometry_msgs::TwistStamped twist;

    float F[6];

    //两个运动点换着注释一下，注释第二点能够在第一个位置测试施加砝码后是否准确
    myController.moveToJoint(jnt_left_initial,10,true);
    sleep(3);

    ForceSensor ati_force_sensor;
    ati_force_sensor.ATIsensor.initATI();
    ati_force_sensor.grav_compsen.initCalParam();
    ati_force_sensor.ATIsensor.biasmeasure();
    float f[6];

/* 边走路径点边发送力信息*/
/*    while( ros::ok() )
    {
        //逐个运动至插值点
*//*        for(int i = 0; i < 10; i++)
        {
            joint_points[i].data=jnt_left_initial.data+i*jnt_left_delta.data*0.1;
            myController.moveToJoint(joint_points[i],5,true);
            std::cout<<"Moving"<<std::endl;
            ros::Duration(0.5).sleep();
        }*//*
                myController.getRobotFrame(frame_tool0);
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


        //置零后下一次不再运动，同时不再发布topic
    }*/

/* 运行至初始位置,不断发布Compensate topic */
    myController.moveToJoint(jnt_left_initial,6,true);
    myController.printCurFrame();

    while( ros::ok() )
    {
        myController.getRobotFrame(frame_tool0);
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

    return 0;
}
