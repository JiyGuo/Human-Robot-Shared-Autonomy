//
// Created by jiy on 2021/10/26.
//
#include "ros/ros.h"
#include "dmp/dmp.h"
#include "dmp/DMPPointStamp.h"
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>

#include "tinyxml2.h"

using namespace std;
using namespace tinyxml2;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "dmp_ent");
    ros::NodeHandle n;

    XMLDocument doc;
    doc.LinkEndChild(doc.NewDeclaration("xml version=\"1.0\" encoding=\"UTF-8\""));
    doc.LinkEndChild(doc.NewComment("this is a DMP library data"));
    XMLElement* root=doc.NewElement("DMPs");
    doc.InsertEndChild(root);

    ros::ServiceClient clientLfd = n.serviceClient<dmp::LearnDMPFromDemo>("learn_dmp_from_demo");
    ros::ServiceClient clientPlan= n.serviceClient<dmp::GetDMPPlan>("get_dmp_plan");
    ros::ServiceClient clientActive = n.serviceClient<dmp::SetActiveDMP>("set_active_dmp");

    ros::Publisher trajPub = n.advertise<dmp::DMPPointStamp>("dmp/trajectory",100);
    ros::Publisher trajDesPub = n.advertise<dmp::DMPPointStamp>("dmp/des_trajectory",100);
    ros::Publisher pathPub = n.advertise<nav_msgs::Path>("xy/trajectory",10, true);
    ros::Publisher pathDesPub = n.advertise<nav_msgs::Path>("xy/des_trajectory",10, true);

    nav_msgs::Path path;
    path.header.frame_id="world";
    geometry_msgs::PoseStamped this_pose_stamped;
    this_pose_stamped.header.frame_id = "world";

    nav_msgs::Path pathDes;
    pathDes.header.frame_id="world";

    dmp::LearnDMPFromDemo srv;
    std::vector<dmp::DMPPoint> points;
    std::vector<double> times;

    double t;
    for(int i=0;i<100;i++){
        t = i*0.05;
        times.push_back(t);
        dmp::DMPPoint xpoint;
        xpoint.positions.push_back(cos(0.2*M_PI*t));
        xpoint.velocities.push_back(-0.2*M_PI*sin(0.2*M_PI*t));
        // y axis
        xpoint.positions.push_back(cos(0.5*M_PI*t));
        xpoint.velocities.push_back(-0.1*M_PI*sin(0.5*M_PI*t));
        points.push_back(xpoint);
        // std::cout<< cos(0.2*M_PI*t) <<" , "<< -0.2*M_PI*sin(0.2*M_PI*t) <<std::endl;
    }

    double k_gain = 25;
    double d_gain = k_gain/4;

    srv.request.k_gains = std::vector<double>{k_gain,k_gain};
    srv.request.d_gains = std::vector<double>{d_gain,d_gain};

    srv.request.num_bases = 100;
    srv.request.demo.points = points;
    srv.request.demo.times = times;

    string dim = to_string(srv.request.k_gains.size());
    root->SetAttribute("dim",dim.c_str());


    if(clientLfd.call(srv)){
        // sleep(5);
        double tau = srv.response.tau;
        std::vector<dmp::DMPData>  dmpList = srv.response.dmp_list;
        for(int i=0;i<dmpList.size();++i){
            auto dmpData = dmpList[i];
            XMLElement* dmp_dataElement = doc.NewElement( "dmp" );
            string id = to_string(i);
            dmp_dataElement->SetAttribute("id",id.c_str());
            XMLElement* kElement = doc.NewElement( "kgain" );
            kElement->LinkEndChild( doc.NewText( to_string(dmpData.k_gain).c_str() ));
            XMLElement* dElement = doc.NewElement( "dgain" );
            dElement->LinkEndChild( doc.NewText( to_string(dmpData.d_gain).c_str() ));
            dmp_dataElement->SetAttribute("id",id.c_str());
            dmp_dataElement->LinkEndChild(kElement);
            dmp_dataElement->LinkEndChild(dElement);
            XMLElement* weightsElement = doc.NewElement( "weights" );
            weightsElement->SetAttribute("nums", to_string(dmpData.weights.size()).c_str());
            for (int j = 0;j<dmpData.weights.size();++j) {
                XMLElement* weightElement = doc.NewElement( "w" );
                weightElement->SetAttribute("id", to_string(j).c_str());
                weightElement->LinkEndChild( doc.NewText( to_string(dmpData.weights[j]).c_str() ));
                weightsElement->InsertEndChild( weightElement );
            }
            dmp_dataElement->LinkEndChild(weightsElement);
            root->LinkEndChild(dmp_dataElement);
        }
        XMLPrinter printer;
        doc.Print(&printer);
        cout<< printer.CStr() << endl;

        doc.SaveFile("myXML.xml");

//        std::vector<double> w = dmpList[0].weights;
//        for(double i:w){
//            std::cout<<i<<std::endl;
//        }

        dmp::SetActiveDMP srvSetActive;
        dmp::DMPTraj traj;
        srvSetActive.request.dmp_list = dmpList;
        if(clientActive.call(srvSetActive)){
            dmp::GetDMPPlan srvPlan;
            srvPlan.request.x_0.push_back(1.5);srvPlan.request.x_0.push_back(1);
            srvPlan.request.x_dot_0.push_back(0);srvPlan.request.x_dot_0.push_back(0);
            srvPlan.request.t_0 = 0;
            srvPlan.request.goal.push_back(0);srvPlan.request.goal.push_back(0);
            srvPlan.request.goal_thresh.push_back(0.01);srvPlan.request.goal_thresh.push_back(0.01);
            srvPlan.request.seg_length = 20;
            srvPlan.request.tau = tau;
            srvPlan.request.dt = 0.01;
            srvPlan.request.integrate_iter = 1;
            if(clientPlan.call(srvPlan) && srvPlan.response.at_goal){
                traj = srvPlan.response.plan;
            }else{
                ROS_INFO("clientPlan services call failed ! ");
            }
        }
        ROS_INFO("DMP services call successful ! ");
//        cout<<traj.points.size()<<endl;
        dmp::DMPPointStamp cur;
        dmp::DMPPointStamp des;
        ros::Rate rate(100);
//        while(ros::ok()){
            for (int i = 0; i < traj.points.size(); ++i) {
                cur.head.stamp = ros::Time::now();
                cur.positions = traj.points[i].positions[0];
                trajPub.publish(cur);

                double time = i*0.01;
                des.head.stamp = ros::Time::now();
                des.positions = cos(0.2*M_PI*time);
                trajDesPub.publish(des);

                path.header.stamp = ros::Time::now();
                this_pose_stamped.header.stamp = ros::Time::now();
                this_pose_stamped.pose.position.x = traj.points[i].positions[0];
                this_pose_stamped.pose.position.y = traj.points[i].positions[1];
                this_pose_stamped.pose.position.z = 0;
                path.poses.push_back(this_pose_stamped);
                pathPub.publish(path);

                pathDes.header.stamp = ros::Time::now();
                this_pose_stamped.pose.position.x = cos(0.2*M_PI*time);
                this_pose_stamped.pose.position.y = cos(0.5*M_PI*time);
                this_pose_stamped.pose.position.z = 0;
                pathDes.poses.push_back(this_pose_stamped);
                pathDesPub.publish(pathDes);

                rate.sleep();
            }
//        }
    }else {
        ROS_INFO("Failled call DMP services! ");
    }
    return 0;
}