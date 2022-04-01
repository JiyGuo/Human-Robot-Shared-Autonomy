//
// Created by geds on 18-7-20.
//
#include "jacobian.h"
#include "mymath.hpp"

namespace GeometryJacobian {
    /**
     * input:
     * output:
     * **/
    /** 将向量转化为反对称矩阵,以便进行叉乘操作
     *    input:
     *              a ：    3x1 向量
     *    output:
     *              m  :    3x3 反对称矩阵
     * **/
    Eigen::Matrix3d hatm(const Eigen::Vector3d a) {
        Eigen::Matrix3d m;
        m << 0, -a[2], a[1],
             a[2], 0, -a[0],
            -a[1], a[0], 0;
        return m;
    }

    /** 计算Ｌ矩阵
    * 　input:
    *       q_in  -- 6个关节角度
    *       ｒ　　 -- follower 相对　leader　的末端位置
    *   outPut
    *       out_L -- 3x6矩阵
    * **/

    void compute_L_matrix(const KDL::JntArray q_in, const Eigen::Vector3d r, Eigen::Matrix<double, 3, 6> &out_L) {
        std::vector<Eigen::Matrix3d> R;
        std::vector<Eigen::Matrix3d> dR;
        KDL::JntArray q(6);
        R.resize(6);
        dR.resize(6);

        // 各关节角度乘以 -1, 实验验证得到的规律,都相差一个负号...
        Multiply(q_in, -1, q);

        // parent: right_base_link　　    child:  right_shoulder_link　　 旋转轴： Z轴
        R[0] << cos(q(0, 0)), -sin(q(0, 0)), 0,
                sin(q(0, 0)), cos(q(0, 0)), 0,
                0, 0, 1;

        // parent: right_shoulder_link　　child:  right_upper_arm_link　　旋转轴： Y轴  绕Y轴旋转了90°
        R[1] << cos(q(1, 0) - 1.5707963), 0, sin(q(1, 0) - 1.5707963),
                0, 1, 0,
                -sin(q(1, 0) - 1.5707963), 0, cos(q(1, 0) - 1.5707963);

        // parent: right_upper_arm_link　　child:  right_forearm_link 　　旋转轴： Y轴
        R[2] << cos(q(2, 0)), 0, sin(q(2, 0)),
                0, 1, 0,
                -sin(q(2, 0)), 0, cos(q(2, 0));

        // parent: right_forearm_link　　child:  right_wrist_1_link   　　旋转轴： Y轴  绕Y轴旋转了90°
        R[3] << cos(q(3, 0) - 1.5707963), 0, sin(q(3, 0) - 1.5707963),
                0, 1, 0,
                -sin(q(3, 0) - 1.5707963), 0, cos(q(3, 0) - 1.5707963);

        // parent: right_wrist_1_link　　child:  right_wrist_2_link   　　旋转轴： Z轴
        R[4] << cos(q(4, 0)), -sin(q(4, 0)), 0,
                sin(q(4, 0)), cos(q(4, 0)), 0,
                0, 0, 1;

        // parent: right_wrist_2_link　　child:  right_wrist_3_link   　　旋转轴： Y轴
        R[5] << cos(q(5, 0)), 0, sin(q(5, 0)),
                0, 1, 0,
                -sin(q(5, 0)), 0, cos(q(5, 0));

        /// 对各旋转矩阵求导
        dR[0] << -sin(q(0, 0)), -cos(q(0, 0)), 0,
                cos(q(0, 0)), -sin(q(0, 0)), 0,
                0, 0, 1;

        dR[1] << -sin(q(1, 0) - 1.5707963), 0, cos(q(1, 0) - 1.5707963),
                0, 1, 0,
                -cos(q(1, 0) - 1.5707963), 0, -sin(q(1, 0) - 1.5707963);

        dR[2] << -sin(q(2, 0)), 0, cos(q(2, 0)),
                0, 1, 0,
                -cos(q(2, 0)), 0, -sin(q(2, 0));

        dR[3] << -sin(q(3, 0) - 1.5707963), 0, cos(q(3, 0) - 1.5707963),
                0, 1, 0,
                -cos(q(3, 0) - 1.5707963), 0, -sin(q(3, 0) - 1.5707963);

        dR[4] << -sin(q(4, 0)), -cos(q(4, 0)), 0,
                cos(q(4, 0)), -sin(q(4, 0)), 0,
                0, 0, 1;

        dR[5] << -sin(q(5, 0)), 0, cos(q(5, 0)),
                0, 1, 0,
                -cos(q(5, 0)), 0, -sin(q(5, 0));

//        /// 对各旋转矩阵求导
//        dR[0] << sin(q(0,0)), cos(q(0,0)), 0,
//                -cos(q(0,0)), sin(q(0,0)), 0,
//                0, 0, 1;
//
//        dR[1] << sin(q(1,0) - 1.5707963), 0, -cos(q(1,0) - 1.5707963),
//                0, 1, 0,
//                cos(q(1,0) - 1.5707963), 0, sin(q(1,0) - 1.5707963);
//
//        dR[2] << sin(q(2,0)), 0, -cos(q(2,0)),
//                0, 1, 0,
//                cos(q(2,0)), 0, sin(q(2,0));
//
//        dR[3] << sin(q(3,0) - 1.5707963), 0, -cos(q(3,0) - 1.5707963),
//                0, 1, 0,
//                cos(q(3,0) - 1.5707963), 0, sin(q(3,0) - 1.5707963);
//
//        dR[4] << sin(q(4,0)), cos(q(4,0)), 0,
//                -cos(q(4,0)), sin(q(4,0)), 0,
//                0, 0, 1;
//
//        dR[5] << sin(q(5,0)), 0, -cos(q(5,0)),
//                0, 1, 0,
//                cos(q(5,0)), 0, sin(q(5,0));

        out_L.col(0) = dR[0] * R[1] * R[2] * R[3] * R[4] * R[5] * r;
        out_L.col(1) = R[0] * dR[1] * R[2] * R[3] * R[4] * R[5] * r;
        out_L.col(2) = R[0] * R[1] * dR[2] * R[3] * R[4] * R[5] * r;
        out_L.col(3) = R[0] * R[1] * R[2] * dR[3] * R[4] * R[5] * r;
        out_L.col(4) = R[0] * R[1] * R[2] * R[3] * dR[4] * R[5] * r;
        out_L.col(5) = R[0] * R[1] * R[2] * R[3] * R[4] * dR[5] * r;
        std::cout << out_L << std::endl;
    }

    /** 计算旋转矩阵 -- 已验证
    *  input
    *          q_in     :   6x1  左臂或右臂各关节角度
    *  output
    *          R_out    :   6x1  6个Rotation
    * */
    void getRotationMatrix(const KDL::JntArray q_in, std::vector<KDL::Rotation> &R_out) {
        std::vector<Eigen::Matrix3d> R;
        KDL::JntArray q(6);
        R.resize(6);
        /// 各关节角度乘以 -1, 实验验证得到的规律,都相差一个负号...
        Multiply(q_in, -1, q);

        /// parent: right_base_link　　    child:  right_shoulder_link　　 旋转轴： Z轴
        R[0] << cos(q(0, 0)), -sin(q(0, 0)), 0,
                sin(q(0, 0)), cos(q(0, 0)), 0,
                0, 0, 1;

        /// parent: right_shoulder_link　　child:  right_upper_arm_link　　旋转轴： Y轴  绕Y轴旋转了90°
        R[1] << cos(q(1, 0) - 1.5707963), 0, sin(q(1, 0) - 1.5707963),
                0, 1, 0,
                -sin(q(1, 0) - 1.5707963), 0, cos(q(1, 0) - 1.5707963);

        /// parent: right_upper_arm_link　　child:  right_forearm_link 　　旋转轴： Y轴
        R[2] << cos(q(2, 0)), 0, sin(q(2, 0)),
                0, 1, 0,
                -sin(q(2, 0)), 0, cos(q(2, 0));

        /// parent: right_forearm_link　　child:  right_wrist_1_link   　　旋转轴： Y轴  绕Y轴旋转了90°
        R[3] << cos(q(3, 0) - 1.5707963), 0, sin(q(3, 0) - 1.5707963),
                0, 1, 0,
                -sin(q(3, 0) - 1.5707963), 0, cos(q(3, 0) - 1.5707963);

        /// parent: right_wrist_1_link　　child:  right_wrist_2_link   　　旋转轴： Z轴
        R[4] << cos(q(4, 0)), -sin(q(4, 0)), 0,
                sin(q(4, 0)), cos(q(4, 0)), 0,
                0, 0, 1;

        /// parent: right_wrist_2_link　　child:  right_wrist_3_link   　　旋转轴： Y轴
        R[5] << cos(q(5, 0)), 0, sin(q(5, 0)),
                0, 1, 0,
                -sin(q(5, 0)), 0, cos(q(5, 0));

        for (int i = 0; i < 6; i++) {
            for (int j = 0; j < 9; j++) {
                R_out[i].data[j] = R[i](j);
            }
        }
    }

    /** 计算右臂各link相对support的齐次变换矩阵 -- 已验证
    *  input
    *          q_in  :   6x1  右臂各关节角度
    *  output
    *          T     :   6x1  6个Frame
    * */
    void getRightFrameToSupport(const KDL::JntArray q_in, std::vector<KDL::Frame> &T) {
        /*if (T.size() != 7)
            ROS_INFO("gjy");
            while (ros::ok()) {
                ROS_INFO("Please initialize the Arguments 'std::vector<KDL::Frame>& T' to a suitable size, eg equal to %d !!!!", 7);
            }*/
        KDL::Frame rightBaseToSupport = KDL::Frame(KDL::Rotation::RPY(2.3446, 0.0426, -0.0476), KDL::Vector(0.0005, -0.22663, 1.53885));
        std::vector<KDL::Rotation> R;
        R.resize(7);

        getRotationMatrix(q_in, R);
        T[0] = rightBaseToSupport * KDL::Frame(R[0], KDL::Vector(0, 0, 0.089159));            // right_shoulder_link  相对  uu_support

        T[1] = T[0] * KDL::Frame(R[1], KDL::Vector(0, 0.13585, 0));                           // right_upper_arm_link 相对  uu_support

        T[2] = T[1] * KDL::Frame(R[2], KDL::Vector(0, -0.1197, 0.425));                       // right_forearm_link   相对  uu_support

        T[3] = T[2] * KDL::Frame(R[3], KDL::Vector(0, 0, 0.39225));                           // right_wrist_1_link   相对  uu_support

        T[4] = T[3] * KDL::Frame(R[4], KDL::Vector(0, 0.093, 0));                             // right_wrist_2_link   相对  uu_support

        T[5] = T[4] * KDL::Frame(R[5], KDL::Vector(0, 0, 0.09465));                           // right_wrist_3_link   相对  uu_support

        T[6] = T[5] * KDL::Frame(KDL::Rotation::RPY(-1.570796, 0, 0),KDL::Vector(0, 0.0823, 0));// right_tool0        相对  uu_support

    }

    /** 计算右臂各兴趣点相对support的齐次变换矩阵 -- 已验证
*  input
*          q_in  :   6x1  右臂各关节角度
*  output
*          A     :   12x1  12个Frame
* */
    void getRightCtrlpointFrameToSupport(const KDL::JntArray q_in, std::vector<KDL::Frame> &TC) {
        /*if (TC.size() != 7)
            ROS_INFO("gjy");
            while (ros::ok()) {
            ROS_INFO("Please initialize the Arguments 'std::vector<KDL::Frame>& T' to a suitable size, eg equal to %d !!!!", 7);
        }*/
        std::vector<KDL::Rotation> R;
        R.resize(7);
        std::vector<KDL::Frame> T;
        T.resize(7);
        getRightFrameToSupport(q_in, T);

        TC.clear();
        KDL::Frame Temp;
        Temp = T[1];                                                                                   //大臂兴趣点1齐次变换矩阵
        TC.push_back(Temp);
        TC.push_back(TC[0] * KDL::Frame(KDL::Rotation::Identity(), KDL::Vector(0, 0, 0.138)));       //大臂兴趣点2齐次变换矩阵

        TC.push_back( TC[1] * KDL::Frame(KDL::Rotation::Identity(), KDL::Vector(0, 0, 0.149)));        //大臂兴趣点3齐次变换矩阵
        TC.push_back(TC[2] * KDL::Frame(KDL::Rotation::Identity(), KDL::Vector(0, 0, 0.138)));        //大臂兴趣点4齐次变换矩阵

        /*球体大小分界线：前者直径182,后者135*/

        TC.push_back(TC[3] * KDL::Frame(KDL::Rotation::Identity(), KDL::Vector(0, -0.095, 0)));        //大臂兴趣点5齐次变换矩阵
        TC.push_back(T[2] * KDL::Frame(KDL::Rotation::Identity(), KDL::Vector(0, 0, 0.075)));         //小臂兴趣点6齐次变换矩阵
        TC.push_back(TC[5] * KDL::Frame(KDL::Rotation::Identity(), KDL::Vector(0, 0, 0.105)));         //小臂兴趣点7齐次变换矩阵
        TC.push_back(TC[6] * KDL::Frame(KDL::Rotation::Identity(), KDL::Vector(0, 0, 0.105)));         //小臂兴趣点8齐次变换矩阵

        /*以下兴趣点均为腕关节坐标点*/

        TC.push_back(T[3]);
        TC.push_back( T[4]);
        TC.push_back(T[5]);

        /*最后一个兴趣点为末端执行器*/
        TC.push_back(T[6]);

    }

    /** 计算左臂各link相对support的齐次变换矩阵 -- 已验证
    *  input
    *          q_in  :   6x1  左臂各关节角度
    *  output
    *          T     :   6x1  6个Frame
    * */
    void getLeftFrameToSupport(const KDL::JntArray q_in, std::vector<KDL::Frame> &T) {
        if (T.size() != 7)
            while (ros::ok()) {
                ROS_INFO(
                        "Please initialize the Arguments 'std::vector<KDL::Frame>& T' to a suitable size, eg equal to %d !!!!",
                        7);
            }
        KDL::Frame leftBaseToSupport = KDL::Frame(KDL::Rotation::RPY(2.36794, -0.0463692, 3.10086),
                                                  KDL::Vector(0.0173835, 0.216081, 1.53393));
        std::vector<KDL::Rotation> R;
        R.resize(7);

        getRotationMatrix(q_in, R);
        T[0] = leftBaseToSupport * KDL::Frame(R[0], KDL::Vector(0, 0, 0.089159));
        T[1] = T[0] * KDL::Frame(R[1], KDL::Vector(0, 0.13585, 0));
        T[2] = T[1] * KDL::Frame(R[2], KDL::Vector(0, -0.1197, 0.425));
        T[3] = T[2] * KDL::Frame(R[3], KDL::Vector(0, 0, 0.39225));
        T[4] = T[3] * KDL::Frame(R[4], KDL::Vector(0, 0.093, 0));
        T[5] = T[4] * KDL::Frame(R[5], KDL::Vector(0, 0, 0.09465));
        T[6] = T[5] * KDL::Frame(KDL::Rotation::RPY(-1.570796, 0, 0), KDL::Vector(0, 0.0823, 0)); // left_tool0 相对 uu_support
    }

    /** 打印输出各个Frame的x,y,z,r,p,y
    *  input
    *          armFrame  :    各个link的frame
    * */
    void printFrame(const std::vector<KDL::Frame> armFrame) {
        const int num = armFrame.size();
        std::cout << "Print Format--> Frame 1: x y z r p y" << std::endl;
        double r = 0, p = 0, y = 0;
        for (int i = 0; i < num; i++) {
            armFrame[i].M.GetRPY(r, p, y);
            std::cout << "Frame " << i << ": " << armFrame[i].p[0] << " " << armFrame[i].p[1] << " " << armFrame[i].p[2]
                      << " " << r << " " << p << " " << y << " " << std::endl;
        }
    }

    /** 计算右臂的雅克比矩阵
     *    input:
     *           q_in  :   6x1   关节角度
     *    output:
     *           Jac   :   6x6   雅克比矩阵
     * **/
    void JntToJacobianRight(const KDL::JntArray q_in, KDL::Jacobian &Jac) {
        if (Jac.columns() != 6)
            ROS_ERROR("Please initialize the number of columns in Jacobian equals to 6 !!!");
        std::vector<Eigen::Vector3d> rot_axis;
        rot_axis.resize(6);
        std::vector<KDL::Frame> T;
        T.resize(7);
        std::vector<Eigen::Vector3d> origin_pos;
        origin_pos.resize(6);
        Eigen::Vector3d ee_pos;
        Eigen::Matrix<double, 6, 6> jac_matrix;

        /// 获取右臂各旋转轴 和 各link坐标系原点坐标
        getRightFrameToSupport(q_in, T);
        for (int i = 0; i < 6; i++) {
            if (i == 0 || i == 4)
                rot_axis[i] << T[i].M.UnitZ().x(), T[i].M.UnitZ().y(), T[i].M.UnitZ().z();  // 旋转轴为Z轴
            else
                rot_axis[i] << T[i].M.UnitY().x(), T[i].M.UnitY().y(), T[i].M.UnitY().z();  // 旋转轴为Y轴

            origin_pos[i] << T[i].p[0], T[i].p[1], T[i].p[2];                               // 各link坐标原点位置
        }
        ee_pos << T[6].p[0], T[6].p[1], T[6].p[2];                                          // right_tool坐标原点位置

        // 计算末端雅克比矩阵
        for (int i = 0; i < 6; i++) {
            jac_matrix.col(i).segment(0, 3) = hatm(rot_axis[i]) * (ee_pos - origin_pos[i]);    // 计算向量积 jac_matrix(0:2,i) = rot_axis[i] x (ee_pos - origin_pos[i])
            jac_matrix.col(i).segment(3, 3) = rot_axis[i];                                     //          jac_matrix(3:5,i) = rot_axis[i]
        }
        Jac.data = jac_matrix;
        //    std::cout << jac_matrix.col(0).segment(0,6) << std::endl;
        //    std::cout << Jac.getColumn(0) << std::endl;
    }

    /** 计算右臂控制点的雅克比矩阵
 *    input:
 *           q_in  :   6x1   关节角度
 *    output:
 *           Jc   :   12x1   12个控制点雅克比矩阵
 * **/
    void JntToJacobianRightCtrlpoint(const KDL::JntArray q_in, std::vector<KDL::Jacobian> &Jc) {
        std::vector<Eigen::Vector3d> rot_axis;
        rot_axis.resize(6);
        std::vector<KDL::Frame> T;
        std::vector<KDL::Frame> TC;
        T.resize(7);
        TC.resize(12);
        Jc.resize(12);
        std::vector<Eigen::Vector3d> origin_pos;
        std::vector<Eigen::Vector3d> control_pos;
        origin_pos.resize(6);
        control_pos.resize(12);
        Eigen::Vector3d ee_pos;
        //Eigen::Matrix<double, 6, 6> jc_matrix ;

        /// 获取右臂各旋转轴 和 各控制点坐标系原点坐标
        getRightFrameToSupport(q_in, T);
        getRightCtrlpointFrameToSupport(q_in, TC);
        for (int i = 0; i < 6; i++) {
            if (i == 0 || i == 4)
                rot_axis[i] << T[i].M.UnitZ().x(), T[i].M.UnitZ().y(), T[i].M.UnitZ().z();  // 旋转轴为Z轴
            else
                rot_axis[i] << T[i].M.UnitY().x(), T[i].M.UnitY().y(), T[i].M.UnitY().z();  // 旋转轴为Y轴

            origin_pos[i] << T[i].p[0], T[i].p[1], T[i].p[2];                               // 各link坐标原点位置
        }
        for (int j = 0; j < 12; j++) {

            control_pos[j] << TC[j].p[0], TC[j].p[1], TC[j].p[2];                               // 各link坐标原点位置

        }

        // 计算控制点雅克比矩阵
        for (int j = 0; j < 12; j++) {
            if (j < 5) {
                Eigen::Matrix<double, 6, 2> jc_matrix;

                for (int i = 0; i < 2; i++) {

                    jc_matrix.col(i).segment(0, 3) = hatm(rot_axis[i]) * (control_pos[j] - origin_pos[i]);    // 计算向量积 jac_matrix(0:2,i) = rot_axis[i] x (control_pos[j] - origin_pos[i])
                    jc_matrix.col(i).segment(3, 3) = rot_axis[i];                                     //          jac_matrix(3:5,i) = rot_axis[i]
                }

                Jc[j].resize(2);       // 一定要给定jacobian矩阵的列数!!!
                Jc[j].data = jc_matrix;
            }

            else if (j < 8) {
                Eigen::Matrix<double, 6, 3> jc_matrix;
                for (int i = 0; i < 3; i++) {
                    jc_matrix.col(i).segment(0, 3) = hatm(rot_axis[i]) * (control_pos[j] - origin_pos[i]);    // 计算向量积 jac_matrix(0:2,i) = rot_axis[i] x (control_pos[j] - origin_pos[i])
                    jc_matrix.col(i).segment(3, 3) = rot_axis[i];                                     //          jac_matrix(3:5,i) = rot_axis[i]
                }
                Jc[j].resize(3);       // 一定要给定jacobian矩阵的列数!!!
                Jc[j].data = jc_matrix;
            }
            else if (j == 8) {
                Eigen::Matrix<double, 6, 4> jc_matrix;
                for (int i = 0; i < 4; i++) {

                    jc_matrix.col(i).segment(0, 3) = hatm(rot_axis[i]) * (control_pos[j] - origin_pos[i]);    // 计算向量积 jac_matrix(0:2,i) = rot_axis[i] x (control_pos[j] - origin_pos[i])
                    jc_matrix.col(i).segment(3, 3) = rot_axis[i];                                     //          jac_matrix(3:5,i) = rot_axis[i]

                }
                Jc[j].resize(4);       // 一定要给定jacobian矩阵的列数!!!
                Jc[j].data = jc_matrix;
            }
            else if(j == 9) {
                Eigen::Matrix<double, 6, 5> jc_matrix;
                for (int i = 0; i < 5; i++) {

                    jc_matrix.col(i).segment(0, 3) = hatm(rot_axis[i]) * (control_pos[j] - origin_pos[i]);    // 计算向量积 jac_matrix(0:2,i) = rot_axis[i] x (control_pos[j] - origin_pos[i])
                    jc_matrix.col(i).segment(3, 3) = rot_axis[i];                                     //          jac_matrix(3:5,i) = rot_axis[i]

                }
                Jc[j].resize(5);       // 一定要给定jacobian矩阵的列数!!!
                Jc[j].data = jc_matrix;
            }
            else{
                Eigen::Matrix<double, 6, 6> jc_matrix;
                for (int i = 0; i < 6; i++) {

                    jc_matrix.col(i).segment(0, 3) = hatm(rot_axis[i]) * (control_pos[j] - origin_pos[i]);    // 计算向量积 jac_matrix(0:2,i) = rot_axis[i] x (control_pos[j] - origin_pos[i])
                    jc_matrix.col(i).segment(3, 3) = rot_axis[i];                                     //          jac_matrix(3:5,i) = rot_axis[i]

                }
                Jc[j].resize(6);       // 一定要给定jacobian矩阵的列数!!!
                Jc[j].data = jc_matrix;
            }


        }

    }

    /** 计算左臂的雅克比矩阵
    *    input:
    *           q_in  :   6x1   关节角度
    *    output:
    *           Jac   :   6x6   雅克比矩阵
    */
    void JntToJacobianLeft(const KDL::JntArray q_in, KDL::Jacobian &Jac) {
        if (Jac.columns() != 6)
            ROS_ERROR("Please initialize the number of columns in Jacobian equals to 6 !!!");
        std::vector<Eigen::Vector3d> rot_axis;
        rot_axis.resize(6);
        std::vector<KDL::Frame> T;
        T.resize(7);
        std::vector<Eigen::Vector3d> origin_pos;
        origin_pos.resize(6);
        Eigen::Vector3d ee_pos;
        Eigen::Matrix<double, 6, 6> jac_matrix;

        /// 获取左臂各旋转轴 和 各link坐标系原点坐标
        getLeftFrameToSupport(q_in, T);
        for (int i = 0; i < 6; i++) {
            if (i == 0 || i == 4)
                rot_axis[i] << T[i].M.UnitZ().x(), T[i].M.UnitZ().y(), T[i].M.UnitZ().z();  // 旋转轴为Z轴
            else
                rot_axis[i] << T[i].M.UnitY().x(), T[i].M.UnitY().y(), T[i].M.UnitY().z();  // 旋转轴为Y轴

            origin_pos[i] << T[i].p[0], T[i].p[1], T[i].p[2];                               // 各link坐标原点位置
        }
        ee_pos << T[6].p[0], T[6].p[1], T[6].p[2];                                          // right_tool0坐标原点位置

        // 计算雅克比矩阵
        for (int i = 0; i < 6; i++) {
            jac_matrix.col(i).segment(0, 3) = hatm(rot_axis[i]) * (ee_pos -
                                                                   origin_pos[i]);    // 计算向量积 jac_matrix(0:2,i) = rot_axis[i] x (ee_pos - origin_pos[i])

            jac_matrix.col(i).segment(3,
                                      3) = rot_axis[i];                                   //          jac_matrix(3:5,i) = rot_axis[i]
        }

        Jac.data = jac_matrix;
    }

    /** 根据末端线速度和角速度更新末端位置
     *  input   :
     *              frame_   :  4x4    当前时刻的末端位置
     *              t        :  6x1    当前时刻的末端速度(vel,rot)
     *              tStep    :         时间步长
     *  output  :
     *              outFrame :  4x4    下一时刻的末端位置
     * **/
    KDL::Frame upDateFrame(const KDL::Frame frame_, const KDL::Twist t, const double tStep) {
        /** 更新Rotation **/
        // 末端角速度
        KDL::Frame outFrame;
        KDL::Rotation R = frame_.M;

        Eigen::Vector3d omega;
        omega << t.rot.x(), t.rot.y(), t.rot.z();
        //std::cout << "omega: " << t.rot.x() << " " << t.rot.y() << " " << t.rot.z() << std::endl;

        //
        Eigen::Vector3d d_phi_;
        d_phi_ = omega.array() * tStep;
        KDL::Rotation dr;
        if (d_phi_.norm() < 10e-4) {
            dr = KDL::Rotation::Identity();
        } else {
            Eigen::Vector3d w_normed = d_phi_ / d_phi_.norm();
            double phi = d_phi_.norm();

            Eigen::Matrix3d dR = Eigen::Matrix3d::Identity() + hatm(w_normed) * sin(phi) +
                                 hatm(w_normed) * hatm(w_normed) * (1 - cos(phi));

            dr = KDL::Rotation(dR(0, 0), dR(0, 1), dR(0, 2),
                               dR(1, 0), dR(1, 1), dR(1, 2),
                               dR(2, 0), dR(2, 1), dR(2, 2));
        }
        outFrame.M = dr * R;

        double r, p, y;
        R.GetRPY(r, p, y);
        //std::cout << "R-r,p,y: " << r << " " << p << " " << y << " " << std::endl;
        outFrame.M.GetRPY(r, p, y);
        //std::cout << "outFrame.M--r,p,y: " << r << " " << p << " " << y << " " << std::endl;

        /** 更新 position **/
        for (int i = 0; i < 3; i++) {
            outFrame.p[i] = frame_.p[i] + tStep * t.vel[i];
        }

        return outFrame;
    }

    /** 根据末端线速度和角速度更新末端旋转矩阵
     *  input   :
     *              R        :  3x3    当前时刻的末端旋转矩阵
     *              t        :  6x1    当前时刻的末端速度(vel,rot)
     *              tStep    :         时间步长
     *  output  :
     *              outFrame :  4x4    下一时刻的末端旋转矩阵
     **/
    KDL::Rotation upDateRotation(const KDL::Rotation R, const KDL::Twist t, const double tStep) {
        // 末端角速度
        Eigen::Vector3d omega;
        omega << t.rot.x(), t.rot.y(), t.rot.z();

        //
        Eigen::Vector3d d_phi_;
        d_phi_ = omega.array() * tStep;
        KDL::Rotation dr;
        if (d_phi_.norm() < 10e-4) {
            dr = KDL::Rotation::Identity();
        } else {
            Eigen::Vector3d w_normed = d_phi_ / d_phi_.norm();
            double phi = d_phi_.norm();

            Eigen::Matrix3d dR = Eigen::Matrix3d::Identity() + hatm(w_normed) * sin(phi) +
                                 hatm(w_normed) * hatm(w_normed) * (1 - cos(phi));

            dr = KDL::Rotation(dR(0, 0), dR(0, 1), dR(0, 2),
                               dR(1, 0), dR(1, 1), dR(1, 2),
                               dR(2, 0), dR(2, 1), dR(2, 2));
        }
        return dr * R;
    }


    /** 求雅克比矩阵的伪逆
     *  input   :
     *              Jac        :  6x6    雅克比矩阵
     *  output  :
     *              jac_inv    :  4x4    雅克比矩阵的伪逆
    **/
    KDL::Jacobian pinv(const KDL::Jacobian Jac) {
        KDL::Jacobian jac_inv;
        Eigen::Matrix<double, 6, 6> jac_matrix;
        jac_matrix = Jac.data;
        jac_inv.data = (jac_matrix.transpose() * jac_matrix).inverse() * jac_matrix.transpose();
        return jac_inv;
    }

    /** 求任意雅克比矩阵的伪逆
 *  input   :
 *              Jac        :  6x6          雅克比矩阵
 *  output  :
 *              jac_inv    :  Dynamicx6    雅克比矩阵的伪逆
**/
    Eigen::Matrix<double,Eigen:: Dynamic, 6> jac_pinv(const KDL::Jacobian Jac) {
        Eigen::Matrix<double, Eigen:: Dynamic, 6> jac_pinv;
        Eigen::Matrix<double, 6, Eigen:: Dynamic> jac_matrix;
        jac_matrix = Jac.data;
        jac_pinv = (jac_matrix.transpose() * jac_matrix).inverse() * jac_matrix.transpose();
        return jac_pinv;
    }

    /** 求雅克比矩阵的逆
     *  input   :
     *              Jac        :  6x6    雅克比矩阵
     *  output  :
     *              jac_inv    :  6x6    雅克比矩阵的逆
    **/
    KDL::Jacobian inv(const KDL::Jacobian Jac) {
        KDL::Jacobian jac_inv;
        Eigen::Matrix<double, 6, 6> jac_matrix;
        jac_matrix = Jac.data;
        jac_inv.data = jac_matrix.inverse();
        return jac_inv;
    }

    /** 根据雅克比矩阵和末端速度求解关节角速度
     *  input   :
     *              Jac        :  6x6    雅克比矩阵
     *              twist      :  6x1    末端速度
     *  output  :
     *              jnt_out    :  6x1    关节角速度
    **/
    void jac_inv_multiply_twist(const KDL::Jacobian Jac, const KDL::Twist twist, KDL::JntArray &jnt_out) {
        Eigen::Matrix<double, 6, 6> jac_matrix = Jac.data;
        Eigen::Matrix<double, 6, 1> twist_vec;
        twist_vec << twist.vel.x(), twist.vel.y(), twist.vel.z(), twist.rot.x(), twist.rot.y(), twist.rot.z();
        jnt_out.data = jac_matrix * twist_vec;
    }


    /** 根据雅克比矩阵和末端速度求解关节角速度
  *  input   :
  *              Jac        :  6x6    雅克比矩阵
  *              twist      :  6x1    末端速度
  *  output  :
  *              jnt_out    :  6x41    关节角速度
 **/
    void jac_pinv_multiply_twist(const Eigen::Matrix<double, Eigen::Dynamic, 6> jac_pinv_matrix, const KDL::Twist twist, KDL::JntArray &jnt_out) {
        Eigen::Matrix<double, 6, 1> twist_vec;
        twist_vec << twist.vel.x(), twist.vel.y(), twist.vel.z(), twist.rot.x(), twist.rot.y(), twist.rot.z();
        jnt_out.data = jac_pinv_matrix * twist_vec;
    }
}

/** 根据雅克比矩阵和末端力求解关节扭矩
 *  input   :
 *              Jac        :  nx6    雅克比矩阵
 *              force      :  6x1    末端力
 *  output  :
 *              jnt_out    :  6x1    关节扭矩
**/
void jac_inv_multiply_force(const KDL::Jacobian Jac, const KDL::Twist twist, KDL::JntArray &jnt_out) {
    Eigen::Matrix<double, 6, 6> jac_matrix = Jac.data;
    Eigen::Matrix<double, 6, 1> twist_vec;
    twist_vec << twist.vel.x(), twist.vel.y(), twist.vel.z(), twist.rot.x(), twist.rot.y(), twist.rot.z();
    jnt_out.data = jac_matrix * twist_vec;
}