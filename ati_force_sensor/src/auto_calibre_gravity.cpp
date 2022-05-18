//
// Created by zhaoxin on 18-4-20.
//
//Mutiply yingyongtonogyi .h wenjian
#include <ros/ros.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <actionlib/client/simple_action_client.h>
#include <geometry_msgs/Point.h>
#include "myATI.h"
#include "robotController/robotController.h"

using namespace KDL;
using namespace Eigen;

typedef  actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction>  Client;

std::vector<JntArray> joint_points;

/**
 * 初始化要标定的6个点的位置
 */
void position_init()
{
    for(int i = 0; i < 6; i++)
    {
        joint_points[i].resize(6);
    }
   // 初始位置朝上的标定点顺序
/*
    joint_points[0].data << 3.2094454765319824, -1.4901555220233362, 1.9351162910461426, -1.8857124487506312, -2.336769167576925, 0.9636991024017334;
    joint_points[1].data << 2.952148675918579, -1.254908863698141, 2.100651741027832, -2.5365174452411097, 0.8019240498542786, 4.105287551879883;
    joint_points[2].data << 3.219059467315674, -1.0837920347796839, 1.9572787284851074, -2.580566708241598, 2.3584296703338623, 3.7293481826782227;
    joint_points[3].data << 3.089114189147949, -1.2714288870440882, 2.058473587036133, -2.3623879591571253, -0.7755401770221155, 3.9401001930236816;
    joint_points[4].data << 3.243922233581543, -1.13513690630068, 1.9875364303588867, -2.52421743074526, 2.3468258380889893, 2.2139554023742676;
    joint_points[5].data << 3.0437183380126953, -1.3718903700457972, 2.0261054039001465, -2.180150810872213, -0.7763670126544397, 2.302245616912842;
*/
    //机器人左臂后面6个点重力补偿矩阵
/*
    joint_points[0].data << -0.5540722052203577, -1.507561985646383, 1.6978645324707031, -1.2428372542010706, -0.9546015898333948, 1.2767877578735352;
    joint_points[1].data << -0.3516166845904749, -0.9479349295245569, 1.82365083694458, -2.053011719380514, -4.0057929197894495, 1.3187576532363892;
    joint_points[2].data << -0.37179452577699834, -1.0452435652362269, 1.5076651573181152, 0.7278727293014526, -4.046873394642965, 3.4136085510253906;
    joint_points[3].data << -0.3640659491168421, -1.008012596760885, 1.4790563583374023, 0.8365625143051147, -3.9866722265826624, 1.5354994535446167;
    joint_points[4].data << -0.5780347029315394, -0.9538262526141565, 2.0618181228637695, -3.1442797819720667, -5.372604791318075, 1.5286736488342285;
    joint_points[5].data << -0.5694196859942835, -0.9373129049884241, 2.0453271865844727, -3.1725133101092737, -5.310601417218344, -0.6396306196795862;

*/


//   第一个关节外时的6个标定位置，上面是1关节朝里面
/*
    joint_points[0].data << -0.41866609, -1.33512103,  1.33469517, -1.24456985, -0.85454636,  3.4450705;
    joint_points[1].data << -0.22533422, -1.30980328,  1.64924888, -1.756447,   -3.94990699,  4.15445958;
    joint_points[2].data << -0.22335153, -1.35723086,  1.4831232,  -1.86430835, -2.35284346,  3.6978989;
    joint_points[3].data << -0.42033637, -1.23949444,  1.68055136, -2.36015639, -5.44383902,  4.41837081;
    joint_points[4].data << -0.42033637, -1.23949444,  1.68055136, -2.36015639, -5.44383902,  5.98916714;
    joint_points[5].data << -0.42033637, -1.23949444,  1.68055136, -2.36015639, -5.44383902,  2.84757449;
*/
    joint_points[0].data << -0.1777260939227503, -1.3020899931537073, 1.7219476699829102, -1.8810036818133753, -0.8023150602923792, 3.761237382888794;
    joint_points[1].data << -0.0216906706439417, -1.1390350500689905, 1.8053336143493652, -2.2803500334369105, -3.9376710096942347, 3.8737127780914307;
    joint_points[2].data << -0.08384925523866826, -1.3202584425555628, 1.6138315200805664, -1.8936899344073694, -2.3667076269732874, 3.8935954570770264;
    joint_points[3].data << -0.06661731401552373, -1.0095332304583948, 1.8924708366394043, -2.4675944487201136, -5.5074748436557215, 3.9373362064361572;
    joint_points[4].data << -0.2236083189593714, -1.1471546331988733, 1.8561391830444336, -2.4501050154315394, -5.493656281624929, 5.729092597961426;
    joint_points[5].data << -0.23820335069765264, -1.1573579947101038, 1.8483762741088867, -2.4461329619037073, -5.491129521523611, 2.607614278793335;}


int main(int argc, char **argv)
{

    myATI ati_force_sensor;     // 定义一个力传感器
    ati_force_sensor.initATI(); // 初始化此力传感器
    ati_force_sensor.biasmeasure();

    ros::init(argc, argv, "calibre_gravity");
    ros::NodeHandle *n;
    n = new ros::NodeHandle();
    robotController Auto_Gravity(n,"right", false);
    //robotController Auto_Gravity(n,"right",true);
    ros::AsyncSpinner spinner(2);
    spinner.start();
    // 等待一下机器人joint_states消息，更新到当前位置
    ros::Duration(4.0).sleep();
    joint_points.resize(6);

    position_init();    // 初始化位置

    MatrixXd current_force(6, 6);

    current_force.setZero();
    for(int point_num = 0; point_num < 6; point_num++)
    {
        // 开始执行
        ROS_INFO("the %d points", point_num);

        std::cout << joint_points[point_num].data << std::endl;
        Auto_Gravity.moveToJoint(joint_points[point_num],10,true);

        sleep(10);
        // 读取当前力传感器的值
        for(int read_times = 0; read_times < 2000; read_times++)
        {

            current_force.block(point_num, 0, 1, 6) =  current_force.block(point_num, 0, 1, 6) + ati_force_sensor.forcemeasure().transpose();
            usleep(10);
        }
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
    return 0;
}