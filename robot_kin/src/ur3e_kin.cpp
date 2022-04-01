#include "ur3e_kin.hpp"
#include "ur_kin.h"

using namespace KDL;
using namespace sensor_msgs;
using namespace ur_kinematics;
using namespace Eigen;
using namespace std;

JointState InitJointState(JntArray initjnt)
{
    // 初始化
    JointState joint = JointState();
    joint.header = std_msgs::Header();
    joint.header.stamp = ros::Time::now();
    // 设定各关节名称
    joint.name.push_back("shoulder_pan_joint");
    joint.name.push_back("shoulder_lift_joint");
    joint.name.push_back("elbow_joint");
    joint.name.push_back("wrist_1_joint");
    joint.name.push_back("wrist_2_joint");
    joint.name.push_back("wrist_3_joint");

    // 初始化关节角度
    joint.position.resize(joint.name.size());
    joint.velocity.resize(joint.name.size());

    for(unsigned int i = 0; i < joint.name.size(); i++)
    {
        joint.position[i] = initjnt.data[i];
    }
    return joint;
}



inline void copyDataFromJntArrayToArray(KDL::JntArray jnt, double *q)
{
    for(int i = 0; i < 6; i++)
    {
        *(q + i) = jnt.data(i);
    }
}

inline void copyDataFromArrayToJntArray(double *q, KDL::JntArray &jnt_out)
{
    for(int i = 0; i < 6; i++)
    {
        jnt_out.data[i] = *(q + i);
    }
}

inline void copyDataFromFrameToArray(KDL::Frame frame_obj, double *T)
{
    for(int i = 0; i < 3; i++)
    {
        // 获取旋转矩阵
        *(T + i) = frame_obj.M.data[i];
        *(T + 4 + i) = frame_obj.M.data[3 + i];
        *(T + 8 + i) = frame_obj.M.data[6 + i];

        *(T + 4*(i + 1) - 1) = frame_obj.p.data[i];     // 获取位置量
    }
    *(T + 15) = 1;
}

inline void copyDataFromArrayToFrame(double *T, KDL::Frame &frame_obj)
{
    for(int i = 0; i < 3; i++)
    {
        // 获取旋转矩阵
        frame_obj.M.data[i] = *(T + i);
        frame_obj.M.data[3 + i] = *(T + 4 + i);
        frame_obj.M.data[6 + i] = *(T + 8 + i);

        frame_obj.p.data[i] = *(T + 4*(i + 1) - 1);     // 获取位置量
    }
}
/**
 * 无参数的构造函数，设置基坐标和工具坐标系
 */
Robot_Kinematics_Annalytical::Robot_Kinematics_Annalytical()
{
    baseFrame = KDL::Frame(KDL::Rotation::RPY(2.3446,0.0426,-0.0476), KDL::Vector(0.0005, -0.22663, 1.538885));
    toolFrame = KDL::Frame(KDL::Rotation::RPY(-M_PI_2, 0, -M_PI_2), KDL::Vector(0.0, 0.0, 0));
//    toolFrame = KDL::Frame(KDL::Rotation::RPY(-M_PI_2, 0, -M_PI_2), KDL::Vector(0.0, 0.0, 0.0)) *
//    KDL::Frame(KDL::Rotation::RPY(0, 0, 0), KDL::Vector(0.0, 0.0, 0.15));
}

void Robot_Kinematics_Annalytical::FKine(KDL::JntArray jnt_in, KDL::Frame &frame_out)
{
    double T[16] = {0};
    double q[6];
    KDL::Frame frame_temp = Frame();
    copyDataFromJntArrayToArray(jnt_in, q);
    forward(q, T);
    copyDataFromArrayToFrame(T, frame_temp);
    frame_out = baseFrame * frame_temp * toolFrame; // 与TF计算结果相同
}

void Robot_Kinematics_Annalytical::IK_analytical(JntArray jnt_init, Frame frame, JntArray &jnt_out)
{
    double T[16] = {0};
    double q[8 * 6] = {0};
    int qnum;
    Frame frame_temp;
    frame_temp = baseFrame.Inverse() * frame * toolFrame.Inverse(); // 与TF计算结果相同
    copyDataFromFrameToArray(frame_temp, T);
    qnum = inverse(T, q);

    IKSelection(q, qnum, jnt_init, jnt_out);
}

int Robot_Kinematics_Annalytical::IKSelection(double *q_sol, int q_num, KDL::JntArray jnt_init, KDL::JntArray &jnt_out)
{
    double q_diff[8] = {0};       // 逆解到初始解的距离，此值辅助进行选解
    double min_dis = 100;        // 保存最小的关节距离值，辅助进行选解
    int min_index = 0;          // 最小距离的索引号，辅助进行选解
    double diff_temp;

    if(q_num > 0)
    {
        for (int i = 0; i < q_num; i++)
        {
            for (int j = 0; j < 6; j++)
            {
                diff_temp = q_sol[i * 6 + j] - jnt_init.data[j];
//                if(j != 2)  // 第三关节由于可能会导致碰撞，因此不能进行翻转
                {
                    if(diff_temp > PI && q_sol[i * 6 + j] > 0)  // 是否满足
                    {
                        q_sol[i * 6 + j] -= 2 * PI;
                        diff_temp -= 2.0 * PI;
                    }
                    else if(diff_temp < -PI && q_sol[i * 6 + j] < 0)
                    {
                        q_sol[i * 6 + j] += 2 * PI;
                        diff_temp += 2.0 * PI;
                    }
                }
                q_diff[i] += absv(diff_temp); // 计算各组逆解到关节初始值的距离
            }

            // 找到距离最小的那组解
            if (q_diff[i] < min_dis)
            {
                min_dis = q_diff[i];
                min_index = i;
            }
        }

        // 将距离最小的解保存到jnt_out中去
        for(int i = 0; i < 6; i++)
            jnt_out.data[i] = q_sol[min_index * 6 + i];

        return 1;   // 求解成功，返回1
    }
    else
    {
        return -1;  // 求解失败，返回-1
    }
}




Robot_Kinematics_Annalytical::~Robot_Kinematics_Annalytical(){

}
