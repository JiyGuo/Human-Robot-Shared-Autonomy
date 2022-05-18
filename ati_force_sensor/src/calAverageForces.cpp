#include "myATI.h"
#include "ros/ros.h"
#include <kdl/frames.hpp>
#include <geometry_msgs/TwistStamped.h>
#include <Eigen/Core>

using namespace Eigen;
VectorXd Force;
int averageTimes = 2000;
int count = 0;

void calAccForces(const geometry_msgs::TwistStampedPtr &msg)
{
    if(count < averageTimes)
    {
        //std::cout << "X: " << msg->twist.linear.x <<  "Y: " << msg->twist.linear.y "Z: " << msg->twist.linear.x
        Force[0] += msg->twist.linear.x/ (double) averageTimes;
        Force[1] += msg->twist.linear.y/ (double) averageTimes;
        Force[2] += msg->twist.linear.z/ (double) averageTimes;
        Force[3] += msg->twist.angular.x/ (double) averageTimes;
        Force[4] += msg->twist.angular.y/ (double) averageTimes;
        Force[5] += msg->twist.angular.z/ (double) averageTimes;

        std::cout << count;
//        std::cout << "  X: " << msg->twist.linear.x;
//        std::cout << "  Y: " << msg->twist.linear.y;
//        std::cout << "  Z: " << msg->twist.linear.z;
//
//        std::cout << "  WX: " << msg->twist.angular.x;
//        std::cout << "  WY: " << msg->twist.angular.y;
//        std::cout << "  WZ: " << msg->twist.angular.z;
        std::cout << std::endl;

    }
    else if(count == averageTimes)
    {
//        Force = Force / (double) averageTimes;
        //std::cout << "The average force is:" << Force.transpose() << std::endl;
        for(int i = 0; i < 6; i++)
            std::cout << Force[i] << ", ";

        std::cout << std::endl;
    }
    count++;

}

int main(int argc, char **argv)
{
    Force.resize(6);
    Force.setZero();

    ros::init (argc, argv, "calAverageForces");
    ros::NodeHandle node_handle;
    ros::AsyncSpinner spinner(1);
    spinner.start();
    ros::Subscriber sub = node_handle.subscribe("ATI_force", 100, calAccForces);

    ros::spin();

    return 1;
}