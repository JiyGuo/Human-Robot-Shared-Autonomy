//
// Created by zhaoxin on 17-10-25.
//

#ifndef ATI_MYATI_H
#define ATI_MYATI_H

#include "pmd.h"
#include "usb-1608FS-Plus.h"
#include "ftconfig.h"
#include <math.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <stdarg.h>
#include <unistd.h>
#include <sys/time.h>
#include <sys/resource.h>
#include <Eigen/Core>
#include <geometry_msgs/Twist.h>
#include <kdl/frames.hpp>

//using namespace Eigen;
//using namespace geometry_msgs;
//using namespace KDL;

class ATIGravityCompensation{
public:
    KDL::Wrench compensete(KDL::Wrench F, KDL::Rotation R);
    KDL::Wrench compensete(float* F, KDL::Rotation R);
    Eigen::Matrix<double, 6, 1> getCartWrench(float* F, KDL::Rotation R, KDL::Vector p);
    Eigen::Matrix<double, 6, 1> getCartForce(float* F, KDL::Rotation R, KDL::Vector p);
    Eigen::Matrix<double, 6, 1> getCartWrench(KDL::Wrench F, KDL::Rotation R, KDL::Vector p);
    Eigen::Matrix<double, 6, 1> getTCPCartForce(float* F, KDL::Rotation R, KDL::Vector p);
    void initCalParam();
    void initCalParam_dow_init();

protected:
    void calParam(Eigen::MatrixXd);
    void calParam_down_init(Eigen::MatrixXd M);
    Eigen::Vector3d F0;
    Eigen::Vector3d T0;
    Eigen::Vector3d L;
    Eigen::Matrix<double, 6, 6> tcp_trans_mat;
    double G;
};

class myATI
{
public:
    int initATI();
    void forcemeasure(float *FT);
    void biasmeasure();
    Eigen::VectorXd forcemeasure();

protected:
    void readData(float *base);

    float table_AIN[NGAINS_USB1608FS_PLUS][NCHAN_USB1608FS_PLUS][2];
    float forcebiastemp[6];
    float forcebias[6];
    libusb_device_handle *udev;
    Calibration *cal;		  // struct containing calibration information
};

class ForceSensor
{
public:
    ATIGravityCompensation grav_compsen;
    myATI ATIsensor;
    void init();

    KDL::Wrench getCompsenForce(KDL::Rotation R);
    KDL::Wrench getCompsenCartForce(KDL::Frame f);
    Eigen::VectorXd getCompsenCartForceVector(KDL::Frame f);
};

#endif //ATI_MYATI_H
