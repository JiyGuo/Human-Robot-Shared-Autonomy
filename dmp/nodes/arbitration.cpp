
#include "ros/ros.h"
#include <fstream>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PointStamped.h>
#include "std_msgs/Float64MultiArray.h"
#include "dmp/DMPPoint.h"
#include "robotController/robotController.h"
#include "Eigen/Core"
#include "dtw.h"
#include "my_dmp.h"
#include "tinyxml2.h"

const bool IF_SIM = true;

using namespace std;
using namespace tinyxml2;
using namespace myDmp;
using namespace dtw;

typedef vector<int> IVector;
typedef vector<bool> BVector;

// 宏定义
const int N_BASES = 15;             // 高斯核函数数量
const int HZ = 125;                 // 频率
const int N_LIBS=3;                 // 技能库数量
const int DIM=2;                    // dimensiona
const double DT = 1.0 / HZ;         // dt
const double F_M = 30;              // 最大力
// Admittance gain
//const DVector M_D = {300.0,300.0};      // 惯性参数  /best
const DVector M_D = {100.0,100.0};

DVector C_D(DIM,0);  // 阻尼参数
// trajectory information
const DVector x_0 = {0.52 , -0.22};
const DVector goal = {0.44 , -0.38};
const DVector x_dot_0 = {0.0 , 0.0};
const DVector goal_thresh = {0.005 , 0.005};

typedef Eigen::Matrix<double,DIM,DIM> DMatrix;

DVector force_total(DIM,0);
DVector force_robot(DIM,0);
DVector force_human(DIM,0);
DVector taus(N_LIBS,0);
//DVector prob(N_LIBS,0);
dmp::DMPPoint cur_state;
// callback
//void cur_stateCB(const dmp::DMPPoint& point){
//    cur_state.positions = point.positions;
//    cur_state.velocities = point.velocities;
//}

nav_msgs::Path points;
geometry_msgs::PoseStamped this_pose_stamped;



void Visualize(ros::Publisher &pub, double* pos)
{
    points.header.stamp = ros::Time::now();
    this_pose_stamped.header.stamp = ros::Time::now();
    this_pose_stamped.pose.position.x = pos[0];
    this_pose_stamped.pose.position.y = pos[1];
    this_pose_stamped.pose.position.z = pos[2];

    points.poses.push_back(this_pose_stamped);
    pub.publish(points);
}

// force_human callback
void force_humanCB(const std_msgs::Float64MultiArray::ConstPtr& force_){
    for (int i = 0; i < DIM; ++i) {
        force_human[i] = force_->data[i];
    }
}

bool find_path(const DMPTraj& path , double& length,
               int& counter, const double& max_len,
               vector<DVector>& templ_vec) {
    size_t dims = path.points[counter].positions.size();
    DVector cur_pos(dims,0);
    DVector next_pos(dims,0);
    cur_pos = path.points[counter].positions;
    while (length <= max_len) {
        if(counter>=path.points.size()-1){
            return true;
        }
        next_pos = path.points[++counter].positions;
        length += dtw::distance(cur_pos,next_pos);
        templ_vec.push_back(next_pos);
        cur_pos = next_pos;
    }
    return false;
}

void gen_path(const vector<MyDmp*>& my_dmps , vector<DMPTraj>& trajs){
    string xml_path = "/home/jiyguo/ur_ws/src/dmp/data/";
    // Dmp library
    vector<string> xml_temps(N_LIBS);
    xml_temps[1] = xml_path + "dmp11.xml";
    xml_temps[0] = xml_path + "dmp01.xml";
    xml_temps[2] = xml_path + "dmp21.xml";
    double cur_t = 0;
    trajs.resize(N_LIBS);
    DVector total_len(N_LIBS,0);
    for (int i=0;i<N_LIBS;++i) {
        if(my_dmps[i]->Init(xml_temps[i]) ){
            my_dmps[i]->set_tau(taus[i]);
            if(!my_dmps[i]->get_plan(trajs[i],total_len[i],cur_t)){
                throw ;
            }
        }else{
            throw ;
        }
    }
}

// human confidence
double confi_h(const DVector& force_){
    double f = 0;
    for (int i = 0; i < force_.size(); ++i) {
        f += pow(force_[i],2);
    }
    f = sqrt(f);
    double lamda = 2* log(99)/F_M;
    double sigma = lamda * F_M / 2;
    return 1.0/(1+ exp(-lamda*f+sigma));
}

// human confidence
//double arbitration_fun(const double& c_h,const double& c_r){
//    double epsilion = 0.01;
//    double gamma = 2;
//    double eta = 1;
//    double pesi = 1.36;
//    return pesi * (1.0 - 1.0/(1+ exp(-(gamma*c_h) / (c_r+epsilion)+eta)));
//}

double arbitration_fun(const double& c_h,const double& c_r){
    double gamma = 1;
    double pesi = 0.01;
    double delta = 1;
    double para = -gamma* ( 2*(c_h+20*pesi) / (c_r+pesi) - delta );
    double alpha = 1-1/(1+ exp(para));
    return 1.5466*alpha;
}


int main(int argc, char **argv){
    ros::init(argc, argv, "arbitration");
    ros::NodeHandle n;
    ros::Rate rate(HZ);
    // 发布各个dmp隶属度概率
    ros::Publisher probPub = n.advertise<geometry_msgs::PointStamped>("/dmp/probability",10, true);
    ros::Publisher forceDmpPub = n.advertise<std_msgs::Float64MultiArray>("/dmp/force_robot",10, true);

    ros::Publisher pathPub = n.advertise<nav_msgs::Path>("/shared_control/online_trajectory",10, true);;
    vector<ros::Publisher> dmp_paths_pub;
    ros::Publisher dmp_path_pub;

    geometry_msgs::PointStamped prob;
    std_msgs::Float64MultiArray force_msgs;
    // 订阅当前末端期望位置
    ros::Publisher statePub = n.advertise<dmp::DMPPoint>("/shared_control/plan_state",10);
    ros::Subscriber forceHumSub = n.subscribe<std_msgs::Float64MultiArray>("/xb4s/force_human", 10, force_humanCB);

    dmp_path_pub = n.advertise<nav_msgs::Path>("/dmp/trajectory1",10, true);
    dmp_paths_pub.push_back(dmp_path_pub);
    dmp_path_pub = n.advertise<nav_msgs::Path>("/dmp/trajectory2",10, true);
    dmp_paths_pub.push_back(dmp_path_pub);
    dmp_path_pub = n.advertise<nav_msgs::Path>("/dmp/trajectory3",10, true);
    dmp_paths_pub.push_back(dmp_path_pub);

    string argument;
    if(argc>1){
        argument = argv[1];
    }
    string file_name = "/home/jiyguo/ur_ws/src/data/admmitance/"+ argument +".txt";
    std::ofstream my_data(file_name,ios::trunc);
    if (!my_data.is_open())
    {
        cout<<file_name<<endl;
        cout << "Can not open this file" << endl;
        return -1;
    }

    points.header.frame_id="world";
    this_pose_stamped.header.frame_id = "world";

    robotController myController(&n,IF_SIM);
    ros::AsyncSpinner spinner(2);
    spinner.start();
    ros::Duration(1).sleep();

    for (int i = 0; i < N_LIBS; ++i) {
        taus[i] = 10;
    }
    // 创建dmp和dtw对象
    vector<MyDmp*> my_dmps(N_LIBS);
    vector<Dtw*> my_dtws(N_LIBS);
    for (int i = 0; i < N_LIBS; ++i) {
        my_dmps[i] = new MyDmp( x_0, goal, x_dot_0, goal_thresh, N_BASES, DT);
        my_dtws[i] = new Dtw(x_0);
    }
    // 生成库中轨迹
    vector<DMPTraj> trajs(N_LIBS);
    gen_path(my_dmps , trajs);
    KDL::Frame tmp_frame;
    myController.getRobotFrame(tmp_frame);
    for (int j = 0; j < N_LIBS; ++j) {
        string dmps_file = "/home/jiyguo/ur_ws/src/data/dmp"+ to_string(j)+".txt";
        std::ofstream mydata_dmps(dmps_file,ios::trunc);
        for(int i=0;i<trajs[j].times.size();++i){
            tmp_frame.p.x(trajs[j].points[i].positions[0]);
            tmp_frame.p.y(trajs[j].points[i].positions[1]);
            mydata_dmps<<tmp_frame.p.x()<<" "<<tmp_frame.p.y()<<endl;
            Visualize( dmp_paths_pub[j], tmp_frame.p.data);
        }
        mydata_dmps.close();
        points.poses.clear();
    }


    /*
     * 初始化变量
     * */
    IVector templ_counters(N_LIBS,0);
    DVector templ_lens(N_LIBS,0);
    vector<DVector> templ_vecs(N_LIBS);
    bool at_goal = false;
    DVector confidances(N_LIBS);
    DVector dtw_dists(N_LIBS,0);
    DVector dgain(DIM,0);
    DVector sig_vec;
    double length_sig = 0;
    double c_h = 0;
    double c_r = 0;
    double alpha = 0;

    // 前一时刻状态初始化为初始状态
    DMPPoint pre_state;
    pre_state.positions = x_0;pre_state.velocities = x_dot_0;
    DVector couple_term(DIM,0);
    DVector force_dmp;
    KDL::Frame cur_frame;
    KDL::Twist cur_twist;
    KDL::Frame targetFrame;
    targetFrame.M = KDL::Rotation::RotX(M_PI);
    targetFrame.p.x(x_0[0] );
    targetFrame.p.y(x_0[1] );
    targetFrame.p.z(1.1 );
    myController.moveToFrame(myController.homeJoint,targetFrame,5,true);

    DVector jointdd(DIM);
    dmp::DMPPoint plan_state;
    plan_state.positions.resize(DIM);
    plan_state.velocities.resize(DIM);
    cur_state.positions.resize(DIM);
    cur_state.velocities.resize(DIM);
    plan_state.positions = x_0;
    double cur_t=0;
    // Online clasify
    while(ros::ok()){

//        if( cur_t<=2 ){
//            force_human[0] = 5;
//            force_human[1] = -5;
//        }else{
//            force_human[0] = 0;
//            force_human[1] = 0;
//        }

        myController.getRobotFrame(cur_frame);
        myController.getRobotVelocity(cur_twist);
        for (int i = 0; i < DIM; ++i) {
            cur_state.positions[i] = cur_frame.p.data[i];
            cur_state.velocities[i]= cur_twist.vel.data[i];
        }
        length_sig += dtw::distance(cur_state.positions, pre_state.positions);
        pre_state = cur_state;
//        ROS_INFO("~~~~~~~ BEFOR find path ~~~~~~~");
        sig_vec = cur_state.positions;
        for (int i=0;i<N_LIBS;++i) {
            templ_vecs.clear();
            find_path(trajs[i],templ_lens[i],templ_counters[i],length_sig,templ_vecs);
            my_dtws[i]->add_data( templ_vecs , sig_vec);
            dtw_dists[i] = my_dtws[i]->get_distance();
            my_data<<dtw_dists[i]<<" ";
        }
//        ROS_INFO("~~~~~~~ AFTER find path ~~~~~~~");
        my_data<<sig_vec[0]<<" "<<sig_vec[1]<<" ";
        confidances.clear();
        dtw::Confidances(dtw_dists, confidances);
        prob.point.x = confidances[0];
        prob.point.y = confidances[1];
        prob.point.z = confidances[2];
        my_data<<prob.point.x<<" "<<prob.point.y<<" "<<prob.point.z<<" ";

        size_t idx = std::max_element(confidances.begin(), confidances.end()) - confidances.begin();

        my_dmps[idx]->get_dgain(dgain);
//        std::cout<<"templ_counters[idx]: "<<templ_counters[idx]<<std::endl;
//        std::cout<<"trajs[idx].points.size(): "<<trajs[idx].points.size()<<std::endl;

//        ROS_INFO("~~~~~~~ BEFOR get_force ~~~~~~~");
//        at_goal = my_dmps[idx]->get_force(cur_state, couple_term,
//                      (double)templ_counters[idx]/trajs[idx].points.size() * taus[idx], force_dmp);
        cur_t = (double)(templ_counters[idx]+1)/trajs[idx].points.size() * taus[idx];
        at_goal = my_dmps[idx]->get_force(cur_state, couple_term,
                                          cur_t, force_dmp);
//        cur_t+=DT;
//        ROS_INFO("~~~~~~~ AFTER get_force ~~~~~~~");
//        cout<<"idx: "<<idx<<endl;

//        ROS_INFO("~~~~~~~ AFTER at_goals[idx] ~~~~~~~");
        my_data<<force_human[0]<<" "<<force_human[1]<<" ";
        my_data<<force_dmp[0]<<" "<<force_dmp[1]<<" ";

        // 置信度计算与仲裁
        c_h = confi_h(force_human);
        sort(confidances.begin(),confidances.end());
        c_r = confidances[2] - confidances[1];
        alpha = arbitration_fun(c_h,c_r);
        alpha = 0;
        my_data<<c_h<<" "<<c_r<<" "<<alpha<<" ";
//        cout<<"alpha: "<<alpha<<endl;
        for (int i = 0; i < DIM; ++i) {
            force_robot[i] = force_dmp[i] * M_D[i];
            force_total[i] = alpha * force_robot[i] + (1-alpha) * force_human[i];
            C_D[i] = M_D[i] * dgain[i] / taus[idx];
            jointdd[i] = (force_total[i]-C_D[i]*plan_state.velocities[i]) / M_D[i];
            plan_state.velocities[i] = plan_state.velocities[i] + jointdd[i] * DT;
            plan_state.positions[i] = plan_state.positions[i] + plan_state.velocities[i] * DT;
            targetFrame.p.data[i] = plan_state.positions[i];
        }
        my_data<<force_total[0]<<" "<<force_total[1]<<" "<<endl;
//        cout<<"force_robot: "<<force_robot[0]<<" , "<<force_robot[1]<<endl;
//        cout<<"force_human: "<<force_human[0]<<" , "<<force_human[1]<<endl;
//        cout<<"plan_state.positions: "<<plan_state.positions[0]<<" , "<<plan_state.positions[1]<<endl;

        myController.moveToFrame(targetFrame,DT, false);

        Visualize( pathPub, targetFrame.p.data);
        statePub.publish(plan_state);
        if(at_goal){
            plan_state.velocities[0] = 0;
            plan_state.velocities[1] = 0;
            statePub.publish(plan_state);
            break;
        }
        rate.sleep();
    }
    my_data.close();

    // Delete pointer
//    for (int i = 0; i < N_LIBS; ++i) {
//        delete my_dmps[i];
//        my_dmps[i]= nullptr;
//        delete my_dtws[i];
//        my_dtws[i]= nullptr;
//    }
    return 0;
}

