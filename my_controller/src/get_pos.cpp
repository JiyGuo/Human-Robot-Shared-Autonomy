//
// Created by jiyguo on 20-11-1.
//

#include "robotController/robotController.h"
#include <iostream>
using namespace std;
int main(int argc, char** argv){

    ros::init(argc, argv, "Test");
    ros::NodeHandle *nh;
    nh = new ros::NodeHandle();
    robotController myController(nh);

    ros::AsyncSpinner spinner(2);
    spinner.start();
    ros::Duration(3).sleep();

    while(1){
	cout<<"Press Enter."<<endl;
	cin.get();
        ros::Duration(1).sleep();
        myController.printCurFrame();
    }
}


