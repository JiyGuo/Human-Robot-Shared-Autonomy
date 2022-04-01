#include "../include/pinv.h"
// ������Υ��
MatXf pinv(MatXf x)
{
	Eigen::JacobiSVD<MatXf> svd(x, Eigen::ComputeFullU | Eigen::ComputeFullV);
	float  pinvtoler = 1.e-8;
	MatXf singularValues_inv = svd.singularValues();
	for (long i = 0; i < x.cols(); ++i) {
		if (singularValues_inv(i) > pinvtoler)
			singularValues_inv(i) = 1.0 / singularValues_inv(i);
		else singularValues_inv(i) = 0;
	}
	return svd.matrixV()*singularValues_inv.asDiagonal()*svd.matrixU().transpose();
}