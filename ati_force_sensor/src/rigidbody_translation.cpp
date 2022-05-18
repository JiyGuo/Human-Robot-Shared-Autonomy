#include "gravityComp/rigidbody_translation.hpp"
#include "geometry_msgs/PoseStamped.h"
#include "ros/ros.h"
#include "kdl/frames.hpp"
#include "myATI.h"
#include <geometry_msgs/TwistStamped.h>
#include <Eigen/Core>
#include <jntarray.hpp>
#include <sensor_msgs/JointState.h>


using namespace KDL;
Frame end_frame;
Wrench measureForce;


double velocity[6];

Frame lastFrame;
double lastTime;
// 读取当前的力传感器信息
void readCurrentForceSensorSUB(const geometry_msgs::TwistStampedPtr &msg)
{
    measureForce.force.data[0] = msg->twist.linear.x;
    measureForce.force.data[1] = msg->twist.linear.y;
    measureForce.force.data[2] = msg->twist.linear.z;
    measureForce.torque.data[0] = msg->twist.angular.x;
    measureForce.torque.data[1] = msg->twist.angular.y;
    measureForce.torque.data[2] = msg->twist.angular.z;

}


void transformCenter(const geometry_msgs::PoseStampedPtr &msg)
{
    KDL::Twist velTwist;

    Frame rigidbody;
    Frame T;

    rigidbody.M = Rotation::Quaternion(msg->pose.orientation.x, msg->pose.orientation.y,
                                       msg->pose.orientation.z, msg->pose.orientation.w);

    rigidbody.p = Vector(msg->pose.position.x, msg->pose.position.y, msg->pose.position.z);



//    double theta =   2.097933378186688;
//    double r = 0.054940899515283;

    // 2018年03月08日数据
//    double theta = 2.034370549725693;
//    double r =  0.054774486342525;
//
////    Frame forceSensorFrame;
//
//    rigidbody = Frame(Rotation::RotX(PI / 2.0)) * rigidbody;
//    end_frame.M = rigidbody.M * Rotation::RotX(PI / 2.0) * Rotation::RotZ(theta);
//    end_frame.p = rigidbody.p - r * end_frame.M.UnitX()  + (0.028 + 0.011) * end_frame.M.UnitZ();

    double theta = 2.066830368899565;
    end_frame = rigidbody * Frame(Rotation::RotZ(-theta)) * Frame(Rotation::RotX(PI));


    end_frame.p = end_frame.p + (0.028 + 0.011) * end_frame.M.UnitZ();

    end_frame.p = end_frame.p * 1000.0;

//    end_frame = Frame(Rotation::RotX(-PI / 2.0)) * rigidbody;
//    end_frame.M = end_frame.M * Rotation::RotX(PI / 2.0) * Rotation::RotZ(theta);
//    end_frame.p = rigidbody.p - r * end_frame.M.UnitX()  + (0.028 + 0.011) * end_frame.M.UnitZ();

    double currentTime = msg->header.stamp.toSec();

    double dt = currentTime - lastTime;
    velTwist = diff(end_frame, lastFrame, dt);

    velocity[0] = velTwist.vel.x();
    velocity[1] = velTwist.vel.y();
    velocity[2] = velTwist.vel.z();

    velocity[3] = velTwist.rot.x();
    velocity[4] = velTwist.rot.y();
    velocity[5] = velTwist.rot.z();
    lastTime = currentTime;
    lastFrame = end_frame;

}

int main(int argc, char **argv) {

    ATIGravityCompensation gravityCom;
    gravityCom.initCalParam();

    ros::init(argc, argv, "calAverageForces");
    ros::NodeHandle node_handle;
    ros::AsyncSpinner spinner(1);
    spinner.start();
    ros::Rate loop_rate(240);

    ros::Subscriber forceSub = node_handle.subscribe("ATI_force", 10, readCurrentForceSensorSUB);
    ros::Subscriber sub = node_handle.subscribe("vrpn_client_node/RigidBody4/pose", 10, transformCenter);

    ros::Publisher pospub = node_handle.advertise<geometry_msgs::PoseStamped>("/end_frame", 10);
    ros::Publisher pub = node_handle.advertise<geometry_msgs::TwistStamped>("/CompenForce", 10);
    ros::Publisher velpub = node_handle.advertise<geometry_msgs::TwistStamped>("/vel", 10);

    geometry_msgs::TwistStamped twist;
    geometry_msgs::TwistStamped velStamped;
    geometry_msgs::PoseStamped pose;

    Eigen::Matrix<double, 6, 1> cartForce;

    double qx, qy, qz, qw;
    while(ros::ok())
    {
        printf("%f, %f, %f\n", end_frame.M.UnitZ()[0], end_frame.M.UnitZ()[1], end_frame.M.UnitZ()[2]);
        cartForce = gravityCom.getCartWrench(measureForce, end_frame.M, end_frame.p);

        end_frame.M.GetQuaternion(qx, qy, qz, qw);
        pose.pose.position.x = end_frame.p.x();
        pose.pose.position.y = end_frame.p.y();
        pose.pose.position.z = end_frame.p.z();
        pose.pose.orientation.x = qx;
        pose.pose.orientation.y = qy;
        pose.pose.orientation.z = qz;
        pose.pose.orientation.w = qw;
        pose.header.stamp = ros::Time::now();
        pospub.publish(pose);


        twist.twist.linear.x = cartForce(0);
        twist.twist.linear.y = cartForce(1);
        twist.twist.linear.z = cartForce(2);
        twist.twist.angular.x = cartForce(3);
        twist.twist.angular.y = cartForce(4);
        twist.twist.angular.z = cartForce(5);
        twist.header.stamp = ros::Time::now();
        pub.publish(twist);


        velStamped.header.stamp = ros::Time::now();
        velStamped.twist.linear.x = velocity[0];
        velStamped.twist.linear.y = velocity[1];
        velStamped.twist.linear.z = velocity[2];
        velStamped.twist.angular.x = velocity[3];
        velStamped.twist.angular.y = velocity[4];
        velStamped.twist.angular.z = velocity[5];
        velpub.publish(velStamped);

        loop_rate.sleep();
    }

    return 0;
}
