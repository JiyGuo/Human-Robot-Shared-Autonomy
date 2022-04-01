//
// Created by geds on 18-7-20.
//

#ifndef MYDS_PACKAGE_MYJACOBIAN_H
#define MYDS_PACKAGE_MYJACOBIAN_H

#include <Eigen/Dense>
#include <kdl/frames.hpp>
#include <kdl/jntarray.hpp>
#include <iostream>
#include <vector>
#include <ros/ros.h>

namespace GeometryJacobian {

Eigen::Matrix3d hatm(const Eigen::Vector3d a);

void compute_L_matrix(const KDL::JntArray q_in, const Eigen::Vector3d r, Eigen::Matrix<double, 3,6>& out_L);

void getRotationMatrix(const KDL::JntArray q_in, std::vector<KDL::Rotation>& R_out);

void getRightFrameToSupport(const KDL::JntArray q_in, std::vector<KDL::Frame>& T);

void getRightCtrlpointFrameToSupport(const KDL::JntArray q_in,std::vector<KDL::Frame> &TC);

void getLeftFrameToSupport(const KDL::JntArray q_in, std::vector<KDL::Frame>& T);

void printFrame(const std::vector<KDL::Frame> armFrame);

void JntToJacobianRight(const KDL::JntArray q_in, KDL::Jacobian& Jac);

void JntToJacobianRightCtrlpoint(const KDL::JntArray q_in, std::vector<KDL::Jacobian> &Jc);

void JntToJacobianLeft(const KDL::JntArray q_in, KDL::Jacobian& Jac);

void jac_pinv_multiply_twist(const Eigen::Matrix<double,Eigen::Dynamic, 6> jac_matrix, const KDL::Twist twist, KDL::JntArray &jnt_out);

Eigen::Matrix<double,Eigen::Dynamic,6> jac_pinv(const KDL::Jacobian Jac);

KDL::Frame upDateFrame(const KDL::Frame frame_, const KDL::Twist t, const double tStep);

KDL::Rotation upDateRotation(const KDL::Rotation R, const KDL::Twist t, const double tStep);

KDL::Jacobian pinv(const KDL::Jacobian Jac);

KDL::Jacobian inv(const KDL::Jacobian Jac);

void jac_inv_multiply_twist(const KDL::Jacobian Jac, const KDL::Twist twist, KDL::JntArray& jnt_out);

}

#endif //MYDS_PACKAGE_MYJACOBIAN_H
