//
// Created by zhaoxin on 18-4-23.
//

#include "myATI.h"

int main(int argc, char **argv)
{
    ForceSensor ati_forcesensor;
//    ati_forcesensor.init();
    ati_forcesensor.ATIsensor.initATI();

    Eigen::VectorXd force;
    force = ati_forcesensor.ATIsensor.forcemeasure();




    std::cout << force.transpose() << std::endl;
}