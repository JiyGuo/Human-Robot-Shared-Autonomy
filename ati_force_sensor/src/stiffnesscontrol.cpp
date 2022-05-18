//
// Created by zhaoxin on 17-11-1.
//

#include "stiffnessControl.h"

using namespace KDL;

KDL::Wrench measureForce;   // 接收到的力数据
KDL::JntArray jntLeft;      // 左臂的关节位置

void readCurrentJntSUB(const sensor_msgs::JointState &JS)
{
    for(int i = 0; i < 6; i++)
        jntLeft.data[i] = JS.position[i];
}

double strTime;
KDL::Frame frame_last;
KDL::JntArray jnt_last;
ForceSensor forceSensor;

Robot_KIN *rob_kin;

void initVar()
{
    strTime = ros::Time::now().toSec();
    jnt_last = JntArray(6);
    jntLeft = JntArray(6);

    forceSensor.ATIsensor.initATI();        // 初始化MCC、力传感器
    forceSensor.ATIsensor.biasmeasure();    // 读取力传感器信息
    forceSensor.grav_compsen.initCalParam();

    strTime = ros::Time::now().toSec(); // 获取当前时间
}

void stiffnesscontrol()
{
    double dt;
    double curTime;

    KDL::Frame frame_current = KDL::Frame();
    KDL::Frame frame_desired = KDL::Frame();

    curTime = ros::Time::now().toSec();
    dt = curTime - strTime;



}

int main(int argc, char **argv)
{
    initVar();

    ros::init (argc, argv, "calAverageForces");
    ros::NodeHandle node_handle;
    ros::AsyncSpinner spinner(1);
    spinner.start();
    ros::Rate loop_rate(125);

    // 建立机器人的运动学模型
    std::string urdf_param;
    double timeout;
    node_handle.param("timeout", timeout, 0.005);
    node_handle.param("urdf_param", urdf_param, std::string("/robot_description"));
    rob_kin = new Robot_KIN(urdf_param, "uu_support", "left_tool0", "uu_support", "right_tool0", 0.008);

    // 建立订阅器，保存实时的位置
    ros::Subscriber jntSub = node_handle.subscribe("left_joint_states", 10, readCurrentJntSUB);

    while(ros::ok())
    {
        stiffnesscontrol();
        loop_rate.sleep();
    }

    return 1;

}