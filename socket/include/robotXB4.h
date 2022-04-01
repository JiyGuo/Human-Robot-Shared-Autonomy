#include <stdio.h>
#include <stdlib.h>
#include <iostream>
#include <Eigen/Dense>
#include <vector>

#include "pinv.h"
#ifndef Pi
#define Pi 3.1415926
#endif

// ��������Ĺ���
class RobotXB4 {
public:
	Eigen::Matrix<float, 1, 6> qPos;
	Eigen::Matrix<float, 6, 1> qVel;
private:
	Eigen::Matrix<float, 4, 4> camInPam;
	std::vector<Eigen::Matrix<float, 3, 3>> RotationMatrix;
	std::vector<Eigen::Matrix<float, 4, 4>> HomoFrame;
	Eigen::Matrix<float, 6, 6> Jac;
public:
	RobotXB4(Eigen::Matrix<float, 4, 4> Pam);
	void updateQPos(Eigen::Matrix<float, 1, 6> qP);
	void updateQVel(Eigen::Matrix<float, 6, 1> cameraVel);
	void updateQVel2(Eigen::Matrix<float, 6, 1> cameraVel);
private:
	// ����ĵ���ת����
	void getRotationMatrix(Eigen::Matrix<float, 1, 6> q);
	// ����ת��ƽ�Ƶõ�һ����α任����
	Eigen::Matrix<float, 4, 4> RotaToHomo(Eigen::Matrix<float, 3, 3> Rota, Eigen::Matrix<float, 3, 1> Tran)
	{
		Eigen::Matrix<float, 4, 4> HomoMatrix;
		HomoMatrix << Rota(0, 0), Rota(0, 1), Rota(0, 2), Tran(0, 0),
			Rota(1, 0), Rota(1, 1), Rota(1, 2), Tran(1, 0),
			Rota(2, 0), Rota(2, 1), Rota(2, 2), Tran(2, 0),
			0, 0, 0, 1;
		return HomoMatrix;
	};
	// ����õ���α任����
	void gethomoFrameToBase();
	// �����ſɱȾ���
	void getJac();
};