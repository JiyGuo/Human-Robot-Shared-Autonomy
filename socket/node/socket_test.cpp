//
// Created by jiyguo on 2022/3/11.
//
#include "mySocket/mySocket.h"
#include "ros/ros.h"
#include "dmp/DMPPoint.h"
#include "std_msgs/Float64MultiArray.h"

const int HZ = 1000;                                   // 频率
const double DT = 1.0/HZ;
using std::cout;
using std::endl;

typedef std::vector<double> DVector;

DVector send_msg(6,0);
// callback
void stateCB(const dmp::DMPPoint::ConstPtr& point){
    for (int i = 0; i < point->positions.size(); ++i) {
//        send_msg[i] = point->positions[i];
        send_msg[i] = point->velocities[i] * 1000;
    }
}

int main(int argc, char **argv){
    ros::init(argc, argv, "socket_test");
    ros::NodeHandle n;
    ros::Rate rate(HZ);
    ros::Subscriber stateSub = n.subscribe<dmp::DMPPoint>("/shared_control/plan_state",10,stateCB);
    ros::Publisher forcePub = n.advertise<std_msgs::Float64MultiArray>("/xb4s/force_human",10, true);
    std_msgs::Float64MultiArray force;
    force.data.resize(6);

    ros::AsyncSpinner spinner(2);
    spinner.start();
    ros::Duration(1).sleep();

    auto* mySocket = new MySocket();
    DVector resv_msg;
    double cur_t=0;
    while(ros::ok()){
        cur_t+=DT;
//        if(cur_t<10){
//            send_msg[1] = 10;
//        }else{
//            send_msg[1] = 0;
//        }
//        if(cur_t>20) break;
//        cout<<send_msg[1]<<endl;
        ROS_INFO("Velocity: %f , %f",send_msg[0],send_msg[1]);

        if(mySocket->SendAndResv(send_msg, resv_msg)){
            for (int i = 0; i < 6; ++i) {
                force.data[i] = resv_msg[i];
            }
        }else{
            std::cerr<<"Socket error ! "<<std::endl;
            throw ;
        }
//        cout<<force.data[0]<<endl;
        ROS_INFO("Force: %f , %f",resv_msg[0],resv_msg[1]);

        forcePub.publish(force);
        rate.sleep();
    }
    return 0;
}

