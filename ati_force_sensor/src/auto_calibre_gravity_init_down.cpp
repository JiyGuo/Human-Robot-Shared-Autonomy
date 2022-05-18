//
// Created by zhaoxin on 18-4-20.
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

typedef  actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction>  Client;
Client *right_client, *left_client;
trajectory_msgs::JointTrajectoryPoint left_curTra_Joint, left_targetTra_Joint;
control_msgs::FollowJointTrajectoryGoal left_goal;
KDL::Frame left_pos[6]; // 要标定的6个姿态
KDL::Frame left_home_pos;    // 初始位置

std::vector<JntArray> joint_points;

std::string prefix_left = "left_";

Robot_Kinematics_Annalytical *robot_kin; // 定义机器人运动学求解器

const std::string defaultname[6] = {
        "shoulder_pan_joint",
        "shoulder_lift_joint",
        "elbow_joint",
        "wrist_1_joint",
        "wrist_2_joint",
        "wrist_3_joint"
};


/**
 * 保存下当前位置
 * @param js
 */
void left_jointstates_sub_callback(const sensor_msgs::JointState &js)
{
    for(int i = 0; i < 6; i++)
    {
        left_curTra_Joint.positions[i] = js.position[i];
    }
}

/**
 * 初始化要标定的6个点的位置
 */
void position_init()
{
    left_goal.trajectory.joint_names.resize(6);

    for(int i = 0; i < 6; i++)
    {
        left_goal.trajectory.joint_names[i] = prefix_left + defaultname[i];
        joint_points[i].resize(6);
    }
    // 初始位置朝上的标定点顺序
    joint_points[1].data << 3.3808376789093018, -1.2296255270587366, 2.020852565765381, -2.1285374800311487, -2.33256966272463, 1.1177806854248047;
    joint_points[0].data << 3.400653839111328, -1.3432439009295862, 2.182788848876953, -2.162243668233053, 0.8141090273857117, 3.5713424682617188;
    joint_points[2].data << 3.410064935684204, -1.3506057898150843, 2.123138427734375, -2.601189915333883, 2.317289352416992, 3.55922532081604;
    joint_points[3].data << 3.407331705093384, -1.3638461271869105, 2.1307778358459473, -2.5908902327166956, -0.8232258001910608, 4.289371967315674;
    joint_points[4].data << 3.4099929332733154, -1.3504264990436, 2.122958183288574, -2.601177994404928, 2.317277193069458, 1.9881473779678345;
    joint_points[5].data << 3.4188404083251953, -1.3645642439471644, 2.107344150543213, -2.5767696539508265, -0.8272512594806116, 2.733799934387207;
}


int main(int argc, char **argv)
{

    myATI ati_force_sensor;     // 定义一个力传感器
    ati_force_sensor.initATI(); // 初始化此力传感器
    ati_force_sensor.biasmeasure();

    ros::init(argc, argv, "calibre_gravity");
    ros::NodeHandle n;
    ros::Subscriber left_jointstates_sub = n.subscribe("left_joint_states", 10, left_jointstates_sub_callback);
    // 通信频率
    ros::Rate loop_rate(1);
    ros::AsyncSpinner spinner(1);
    spinner.start();

    // 初始化变量
    left_goal.trajectory.joint_names.resize(6);
    left_curTra_Joint.positions.resize(6);
    left_curTra_Joint.velocities.resize(6);
    left_targetTra_Joint.positions.resize(6);
    left_targetTra_Joint.velocities.resize(6);
    joint_points.resize(6);

    // 等待一下机器人joint_states消息，更新到当前位置
    loop_rate.sleep();


    robot_kin = new Robot_Kinematics_Annalytical(); // 建立机器人运动学模型

    left_client = new Client(prefix_left+"follow_joint_trajectory", true);  // 建立action client

    bool left_state = left_client->waitForServer(ros::Duration(4)); //等待action server响应

    if(left_state) ROS_INFO("connected the server.");
    else {ROS_INFO("connect the server failed."); return 0;}

    position_init();    // 初始化位置

    JntArray ik_jnt(6);
    MatrixXd current_force(6, 6);

    current_force.setZero();
    for(int point_num = 0; point_num < 6; point_num++)
    {
        // 清除跟踪轨迹上原来的点
        left_goal.trajectory.points.clear();

        // 添加当前位置
        left_curTra_Joint.time_from_start = ros::Duration(0);
        left_goal.trajectory.points.push_back(left_curTra_Joint);   // 添加到当前点

        // 先将目标位置进行逆解求得关节角
        for(int i = 0; i < 6; i++)
        {
            // 将关节角进行赋值
            left_targetTra_Joint.positions[i] = joint_points[point_num].data[i];
            left_targetTra_Joint.velocities[i] = 0.0;
        }
        // 添加到路径点中
        left_targetTra_Joint.time_from_start = ros::Duration(6);
        left_goal.trajectory.points.push_back(left_targetTra_Joint);
        left_goal.trajectory.header.stamp = ros::Time::now();

        // 开始执行
        left_client->sendGoal(left_goal);
        left_client->waitForResult(ros::Duration(0));   // 等待执行结束

        // 如果没有运行到目标点，则返回失败
        if(left_client->getState() != actionlib::SimpleClientGoalState::SUCCEEDED)
        {
            ROS_WARN("Robot error, cannot reach the goal!");
            return 0;
        }

        // 读取当前力传感器的值
        for(int read_times = 0; read_times < 2000; read_times++)
        {
            current_force.block(point_num, 0, 1, 6) =  current_force.block(point_num, 0, 1, 6) + ati_force_sensor.forcemeasure().transpose();
            usleep(1000);
        }
        ROS_INFO("the %d points", point_num);
    }

    // 求平均值
    current_force = current_force / 2000.0;

    // 输出力传感器值
    for(int i = 0; i < 6; i++) {
        for(int j = 0; j < 6; j++) {
            printf("%lf, ", current_force(i, j));
        }
        printf("\n");
    }


    left_client->cancelGoal();
    return 0;
}