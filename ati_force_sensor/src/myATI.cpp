//
// Created by zhaoxin on 17-10-25.
//
#include <iostream>
#include "myATI.h"

int myATI::initATI()
{
    //char *calfilepath;      // name of calibration file  18913
    char *calfilepath = (char*)"/home/jiyguo/ur_ws/src/ati_force_sensor/src/calibrationfiles/archFT35046.cal";      // name of calibration file

    unsigned short index = 1;   // index of calibration in file (second parameter; default = 1)
    short sts;              // return value from functions
    int ret;
    struct tm calDate;
    int j;
    int i;
    float SampleTT[6]={0,0,0,0,0,0};
    FILE *biasfile;

    libusb_device_handle *udev2;
    i = 0;

    // 读取力初始值，如果读取文件失败，则将所有值置为零
    biasfile = fopen("/home/jiyguo/ur_ws/src/ati_force_sensor/src/calibrationfiles/forcebias.txt", "r");
    if(biasfile == NULL)
    {
        forcebias[0] = 0;
        forcebias[1] = 0;
        forcebias[2] = 0;
        forcebias[3] = 0;
        forcebias[4] = 0;
        forcebias[5] = 0;
    }

    else
    {
        fscanf(biasfile, "%f, %f, %f, %f, %f, %f", &forcebias[0], &forcebias[1]
                , &forcebias[2], &forcebias[3]
                , &forcebias[4], &forcebias[5]);
        fclose(biasfile);
    }

    start:
    udev = NULL;
    udev2 = NULL;

    ret = libusb_init(NULL);
    if (ret < 0)
    {
        perror("usb_device_find_USB_MCC: Failed to initialize libusb");
        exit(1);
    }

    if ((udev = usb_device_find_USB_MCC(USB1608FS_PLUS_PID, NULL)))
    {
        printf("Success, found a USB 1608FS-Plus!\n");
    }
    else
    {
        printf("Failure, did not find a USB 1608FS-Plus!\n");
        return 0;
    }
    /******************************** Finding a second device has issues on the Raspberry Pi **************/
    // See if there is a second device:
#if defined(LIBUSB_API_VERSION) && (LIBUSB_API_VERSION >= 0x01000103)
    if ((udev2 = usb_device_find_USB_MCC(USB1608FS_PLUS_PID, NULL)))
    {
        printf("Success, found a second USB 1608FS-Plus!\n");
    }
    else
    {
        printf("Did not find a second device.\n");
    }
#endif

    // some initialization
    //print out the wMaxPacketSize.  Should be 64.
    printf("wMaxPacketSize = %d\n", usb_get_max_packet_size(udev,0));

    usbBuildGainTable_USB1608FS_Plus(udev, table_AIN);
    for (i = 0; i < NGAINS_USB1608FS_PLUS; i++ )
    {
        for (j = 0; j < NCHAN_USB1608FS_PLUS; j++)
        {
            //每次查看ATI力传感器读数是否存在问题，得放在USB3.0的接口上，不用拓展坞
            printf("Calibration Table: Range = %d Channel = %d Slope = %f   Offset = %f\n",
                   i, j, table_AIN[i][j][0], table_AIN[i][j][1]);
        }
    }
    usbCalDate_USB1608FS_Plus(udev, &calDate);
    printf("\n");

    // create Calibration struct
    cal = createCalibration(calfilepath, index);
    if (cal==NULL)
    {
        printf("\nSpecified calibration could not be loaded.\n");
        scanf(".");
        return 0;
    }
    // Set force units.
    // This step is optional; by default, the units are inherited from the calibration file.
    sts = SetForceUnits(cal, (char*)"N");
    switch (sts)
    {
        case 0: break;	// successful completion
        case 1: printf("Invalid Calibration struct");
            return 0;
        case 2: printf("Invalid force units");
            return 0;
        default: printf("Unknown error");
            return 0;
    }
    // Set torque units.
    // This step is optional; by default, the units are inherited from the calibration file.
    sts = SetTorqueUnits(cal, (char*)"N-m");
    switch (sts)
    {
        case 0: break;	// successful completion
        case 1: printf("Invalid Calibration struct");
            return 0;
        case 2: printf("Invalid torque units");
            return 0;
        default: printf("Unknown error");
            return 0;
    }

    // Set tool transform.
    // This line is only required if you want to move or rotate the sensor's coordinate system.
    // This example tool transform translates the coordinate system 20 mm along the Z-axis
    // and rotates it 45 degrees about the X-axis.
    sts = SetToolTransform(cal,SampleTT,(char*)"mm",(char*)"degrees");
    switch (sts)
    {
        case 0: break;	// successful completion
        case 1: printf("Invalid Calibration struct"); return 0;
        case 2: printf("Invalid distance units"); return 0;
        case 3: printf("Invalid angle units"); return 0;
        default: printf("Unknown error");
            return 0;
    }

    Bias(cal, forcebias);
}


void myATI::readData(float *base)
{
    int i;
    int j;
    int k;

    int nchan;
    int frequency = 1000;	// 采样频率
    int ret;
    uint32_t count = 1;
    uint16_t sdataIn[8*512]; // holds 16 bit unsigned analog input data
    uint16_t data;
    uint8_t range;
    uint8_t ranges[8] = {0, 0, 0, 0, 0, 0, 0, 0};
    uint8_t channels;
    float reading[6];

    float F[6];

    nchan = 6;		//  6个通道
    channels = 0;

    for(i = 0; i < nchan; i++)
    {
        channels |= (1 << i);
    }

    memset(ranges, 0, sizeof(ranges));
    usbAInScanConfig_USB1608FS_Plus(udev, ranges);

    usbAInScanStart_USB1608FS_Plus(udev, count, frequency, channels, 0);

    ret = usbAInScanRead_USB1608FS_Plus(udev, count, nchan, sdataIn, 0);

    if (ret != count * nchan * 2)
    {
        printf("***ERROR***  ret = %d   count = %d  nchan = %d\n", ret, count, nchan);
    } /* if (ret != count * nchan * 2) */

    for (i = 0; i < count; i++)
    {
        for (j = 0; j < nchan; j++)
        {
            k = i * nchan + j;

            data = rint(sdataIn[k] * table_AIN[i][j][0] + table_AIN[i][j][1]);
            *(base + j) = volts_USB1608FS_Plus(data, i);
        } /* for (j - 0; j < 8, j++) */

    } /* for (i = 0; i < count; i++) */

}

/**
 * 在初始位置累计一定点数的力数据，求均值
 */
void myATI::biasmeasure()
{
    float baseTemp[6];
    int readTimes = 5000;

    // 先将bias里面的值清零
    for(int i = 0; i < 6; i++)
        forcebias[i] = 0;

    std::cout << "reading bias, please wait..." << std::endl;
    for(int i = 0; i < readTimes; i++)
    {
        readData(baseTemp);
        for(int j = 0; j < 6; j++)
            forcebias[j] += baseTemp[j];
        std::cout << "baseTemp[1]..." <<baseTemp[1]<< std::endl;

        usleep(500);
    }

    for(int j = 0; j < 6; j++)
        forcebias[j] = forcebias[j] / (float) readTimes;

    std::cout << "bias in readed!" << std::endl;

    Bias(cal, forcebias);

    FILE *biasfile;
    biasfile = fopen("/home/jiyguo/ur_ws/src/ati_force_sensor/src/calibrationfiles/forcebias.txt", "w");

    fprintf(biasfile, "%f, %f, %f, %f, %f, %f", forcebias[0], forcebias[1], forcebias[2], forcebias[3], forcebias[4], forcebias[5]);
    fclose(biasfile);
}

/**
 * 获取力传感器测得的力数据
 * @param FT : 测量得到的力
 */
void myATI::forcemeasure(float *FT)
{
    float reading[6];
    readData(reading);
    ConvertToFT(cal, reading, FT);
}

Eigen::VectorXd myATI::forcemeasure(void)
{
    float reading[6];
    Eigen::VectorXd force(6);
    forcemeasure(reading);
    for(int i = 0; i < 6; i++)
        force[i] = reading[i];
    return force;
}

/**
 * 根据硕士论文《装配机器人作业过程控制系统应用与软件开发》进行重力补偿
 * @param M : 根据论文方法测量的6个姿态下力传感器数据
 */
void ATIGravityCompensation::calParam(Eigen::MatrixXd M)
{
    // 计算初始重力
    F0[0] = (M(2, 0) + M(3, 0)) / 2.0;
    F0[1] = (M(4, 1) + M(5, 1)) / 2.0;
    F0[2] = (M(0, 2) + M(1, 2)) / 2.0;

    // 计算力矩
    T0[0] = (M(0, 3) + M(1, 3) + M(4, 3) + M(5, 3)) / 4.0;
    T0[1] = (M(0, 4) + M(1, 4) + M(2, 4) + M(3, 4)) / 4.0;
    T0[2] = (M(2, 5) + M(3, 5) + M(4, 5) + M(5, 5)) / 4.0;

    G = (M(1, 2) - M(0, 2) + M(3, 0) - M(2, 0) + M(5, 1) - M(4, 1)) / 6.0;

    L[0] = (M(0, 4) - M(1, 4) + M(5, 5) - M(4, 5)) / 4.0 / G;
    L[1] = (M(1, 3) - M(0, 3) + M(2, 5) - M(3, 5)) / 4.0 / G;
    L[2] = (M(3, 4) - M(2, 4) + M(4, 3) - M(5, 3)) / 4.0 / G;
}

/**
 * 根据力传感器测量的力，和力传感器姿态对重力进行补偿
 * @param F     ： 力传感器测得的力
 * @param R     ： 力传感器姿态（注意这里是力传感器姿态不是末端姿态）
 * @return      ： 返回补偿后的力
 */
KDL::Wrench ATIGravityCompensation::compensete(KDL::Wrench F, KDL::Rotation R)
{
//    Twist twist;
    KDL::Wrench w;
    KDL::Vector G_v = G * KDL::Vector(R.data[6], R.data[7], R.data[8]);

    w.force.data[0] = F.force.x() + G_v[0] - F0[0];
    w.force.data[1] = F.force.y() + G_v[1] - F0[1];
    w.force.data[2] = F.force.z() + G_v[2] - F0[2];

    w.torque[0] = F.torque[0] + G_v[2] * L[1] - G_v[1] * L[2] - T0[0];
    w.torque[1] = F.torque[1] + G_v[0] * L[2] - G_v[2] * L[0] - T0[1];
    w.torque[2] = F.torque[2] + G_v[1] * L[0] - G_v[0] * L[1] - T0[2];

    return w;
}

/**
 * 对重力补偿函数进行重载，可以直接利用力传感器测量得到的数组进行计算
 * @param F
 * @param R
 * @return
 */
KDL::Wrench ATIGravityCompensation::compensete(float* F, KDL::Rotation R)
{
    KDL::Wrench w;
    w.force = KDL::Vector(F[0], F[1], F[2]);
    w.torque = KDL::Vector(F[3], F[4], F[5]);

    return compensete(w, R);
}

Eigen::Matrix<double, 6, 1> ATIGravityCompensation::getCartWrench(float* F, KDL::Rotation R, KDL::Vector p)
{
    Eigen::Matrix<double, 6, 1> w;
    KDL::Wrench kw_f;
    KDL::Wrench kw_c;


    kw_f = compensete(F, R);
    kw_c.force = R * kw_f.force;
//    kw_c.torque = p * (R * kw_f.force) + R * kw_f.torque;
    kw_c.torque = R * kw_f.torque;

    w(0) = kw_c.force.x();
    w(1) = kw_c.force.y();
    w(2) = kw_c.force.z();
    w(3) = kw_c.torque.x();
    w(4) = kw_c.torque.y();
    w(5) = kw_c.torque.z();

    return w;

}

Eigen::Matrix<double, 6, 1> ATIGravityCompensation::getCartForce(float* F, KDL::Rotation R, KDL::Vector p)
{
    Eigen::Matrix<double, 6, 1> w;
    KDL::Wrench kw_f;
    KDL::Wrench kw_c;


    kw_f = compensete(F, R);
    kw_c.force = R * kw_f.force;
//    kw_c.torque = p * (R * kw_f.force) + R * kw_f.torque;
    kw_c.torque = R * kw_f.torque;
//    kw_c.torque = kw_f.torque;

    w(0) = kw_c.force.x();
    w(1) = kw_c.force.y();
    w(2) = kw_c.force.z();
    w(3) = kw_c.torque.x();
    w(4) = kw_c.torque.y();
    w(5) = kw_c.torque.z();

    return w;

}

Eigen::Matrix<double, 6, 1> ATIGravityCompensation::getCartWrench(KDL::Wrench F, KDL::Rotation R, KDL::Vector p)
{
    Eigen::Matrix<double, 6, 1> w;
    KDL::Wrench kw_f;
    KDL::Wrench kw_c;


    kw_f = compensete(F, R);
    kw_c.force = R * kw_f.force;
    kw_c.torque = p * (R * kw_f.force) + R * kw_f.torque;

    w(0) = kw_c.force.x();
    w(1) = kw_c.force.y();
    w(2) = kw_c.force.z();
    w(3) = kw_c.torque.x();
    w(4) = kw_c.torque.y();
    w(5) = kw_c.torque.z();

    return w;
}

/**
 * 根据测量得到的数据进行重力补偿，这些数据是离线先测得的。
 */
void ATIGravityCompensation::initCalParam()
{
    Eigen::MatrixXd gravityParamMat;
    gravityParamMat.resize(6, 6);
    gravityParamMat <<0.002643, -0.022689, 0.030677, -0.000054, 0.001308, 0.000622,
            0.010487, -0.028467, 8.558453, 0.025968, 0.017201, 0.001080,
            -4.321248, -0.046405, 4.351846, 0.013007, -0.661367, 0.008317,
            4.358392, -0.045771, 4.246127, 0.018179, 0.681479, -0.004473,
            0.323229, -4.743180, 4.381109, 0.744371, 0.062504, 0.007381,
            -0.348623, 3.883535, 4.253169, -0.600548, -0.042182, 0.000383;



    calParam(gravityParamMat);
}

/**
 * 获取补偿重力后的力
 * @param R     ： 力传感器姿态
 * @return      ： 补偿后的重力
 */
KDL::Wrench ForceSensor::getCompsenForce(KDL::Rotation R)
{
    float F[6];
    KDL::Wrench w;
    ATIsensor.forcemeasure(F);
    return grav_compsen.compensete(F, R);
}

KDL::Wrench ForceSensor::getCompsenCartForce(KDL::Frame f)
{
    float F[6];
    Eigen::VectorXd force;
    ATIsensor.forcemeasure(F);
    force = grav_compsen.getCartWrench(F, f.M, f.p);

    KDL::Wrench w;
    w.force = KDL::Vector(force[0], force[1], force[2]);
    w.torque = KDL::Vector(force[3], force[4], force[5]);
    return w;
}

Eigen::VectorXd ForceSensor::getCompsenCartForceVector(KDL::Frame f)
{
    float F[6];
    Eigen::VectorXd force;
    ATIsensor.forcemeasure(F);
    force = grav_compsen.getCartForce(F, f.M, f.p);

    return force;
}


/**
 * 力传感器初始化
 */
void ForceSensor::init()
{
    ATIsensor.initATI();
//    ATIsensor.biasmeasure();
    grav_compsen.initCalParam();
}
