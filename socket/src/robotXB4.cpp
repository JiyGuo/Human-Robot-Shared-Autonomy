#include "robotXB4.h"

// 构造函数
RobotXB4::RobotXB4(Eigen::Matrix<float, 4, 4> Pam)
{
	camInPam = Pam;
	RotationMatrix = std::vector<Eigen::Matrix<float, 3, 3>>(7);
	HomoFrame = std::vector<Eigen::Matrix<float, 4, 4>>(7);
}
// 更新关节角
void RobotXB4::updateQPos(Eigen::Matrix<float, 1, 6> qP)
{
	qPos << qP[0], qP[1], qP[2], qP[3], qP[4], qP[5];
	getRotationMatrix(qPos);
	gethomoFrameToBase();
	getJac();
}
// 更新关节速度，输入：末端在末端坐标系下的速度
void RobotXB4::updateQVel(Eigen::Matrix<float, 6, 1> cameraVel)
{
	Eigen::Matrix<float, 3, 3> Rota = RotationMatrix[0] * RotationMatrix[1] * RotationMatrix[2] *
		RotationMatrix[3] * RotationMatrix[4] * RotationMatrix[5] * RotationMatrix[6];
	Eigen::Matrix<float, 6, 6> VelTransfor;
	Eigen::Matrix<float, 3, 3> ZeroMatrix;
	ZeroMatrix << 0, 0, 0, 0, 0, 0, 0, 0, 0;
	VelTransfor << Rota, ZeroMatrix, ZeroMatrix, Rota;
	Eigen::Matrix<float, 6, 1> Veltobase = VelTransfor * cameraVel;
	qVel = pinv(Jac)*Veltobase;
}
// 更新关节速度，输入：末端在世界坐标系下的速度
void RobotXB4::updateQVel2(Eigen::Matrix<float, 6, 1> cameraVel)
{
	qVel = pinv(Jac)*cameraVel;
}
// 计算7个旋转矩阵
void RobotXB4::getRotationMatrix(Eigen::Matrix<float, 1, 6> q)
{
	Eigen::Matrix<float, 3, 3> rot1_2, rot3_4, rot4_5, rot5_6;
	// base to link1 
	RotationMatrix[0] << cos(q[0]), -sin(q[0]), 0,
		sin(q[0]), cos(q[0]), 0,
		0, 0, 1;
	// link1 to link2
	rot1_2 << 1, 0, 0,
		0, cos(-Pi / 2), -sin(-Pi / 2),
		0, sin(-Pi / 2), cos(-Pi / 2);
	RotationMatrix[1] << cos(q[1]), -sin(q[1]), 0,
		sin(q[1]), cos(q[1]), 0,
		0, 0, 1;
	RotationMatrix[1] = rot1_2 * RotationMatrix[1];
	// link2 to link3
	RotationMatrix[2] << cos(q[2]), -sin(q[2]), 0,
		sin(q[2]), cos(q[2]), 0,
		0, 0, 1;
	// link3 to link4
	rot3_4 << cos(Pi / 2), 0, sin(Pi / 2),
		0, 1, 0,
		-sin(Pi / 2), 0, cos(Pi / 2);
	RotationMatrix[3] << cos(q[3]), -sin(q[3]), 0,
		sin(q[3]), cos(q[3]), 0,
		0, 0, 1;
	RotationMatrix[3] = rot3_4 * RotationMatrix[3];
	// link4 to link5
	rot4_5 << cos(-Pi / 2), 0, sin(-Pi / 2),
		0, 1, 0,
		-sin(-Pi / 2), 0, cos(-Pi / 2);
	RotationMatrix[4] << cos(q[4]), -sin(q[4]), 0,
		sin(q[4]), cos(q[4]), 0,
		0, 0, 1;
	RotationMatrix[4] = rot4_5 * RotationMatrix[4];
	// link5 to link6
	rot5_6 << cos(Pi / 2), 0, sin(Pi / 2),
		0, 1, 0,
		-sin(Pi / 2), 0, cos(Pi / 2);
	// 3楼机器人平台构型,5关节相差90度
	Eigen::Matrix<float, 3, 3> rotRealEnd_3F;
	rotRealEnd_3F << cos(-Pi / 2), -sin(-Pi / 2), 0,
		sin(-Pi / 2), cos(-Pi / 2), 0,
		0, 0, 1;
	RotationMatrix[5] << cos(q[5]), -sin(q[5]), 0,
		sin(q[5]), cos(q[5]), 0,
		0, 0, 1;
	// vrep仿真环境构型
	// RotationMatrix[5] = rot5_6 * RotationMatrix[5];
	RotationMatrix[5] = rot5_6 * rotRealEnd_3F * RotationMatrix[5];
	// link6 to tool 
	// 相机的坐标系与joint6相差pi/2
	RotationMatrix[6] << camInPam(0, 0), camInPam(0, 1), camInPam(0, 2),
		camInPam(1, 0), camInPam(1, 1), camInPam(1, 2),
		camInPam(2, 0), camInPam(2, 1), camInPam(2, 2);
}
// 计算7个齐次变换矩阵
void RobotXB4::gethomoFrameToBase()
{
	std::vector<Eigen::Matrix<float, 3, 1>> Tran(7);
	// 六个平移量
	Tran[0] << 0, 0, 0;
	Tran[1] << 0.04, 0, 0.342;
	Tran[2] << 0, -0.275, 0;
	Tran[3] << 0, -0.025, 0;
	Tran[4] << 0, 0, 0.28;
	Tran[5] << 0.073, 0, 0;
	Tran[6] << camInPam(0, 3), camInPam(1, 3), camInPam(2, 3); // 仿真中相机与末端重叠
															   // Tran[6] << 0.0495, 0, 0.089; // 机器人真实的安装尺寸
	HomoFrame[0] = RotaToHomo(RotationMatrix[0], Tran[0]);
	for (int i = 1; i < 7; i++)
	{
		HomoFrame[i] = RotaToHomo(RotationMatrix[i], Tran[i]);
		HomoFrame[i] = HomoFrame[i - 1] * HomoFrame[i];
	}
}
// 计算雅可比
void RobotXB4::getJac()
{
	Eigen::Matrix<float, 3, 6> Zi;
	Eigen::Matrix<float, 3, 6> Oi_On;
	for (int i = 0; i < 6; i++)
	{
		Zi(0, i) = HomoFrame[i](0, 2);
		Zi(1, i) = HomoFrame[i](1, 2);
		Zi(2, i) = HomoFrame[i](2, 2);
		Oi_On(0, i) = HomoFrame[6](0, 3) - HomoFrame[i](0, 3);
		Oi_On(1, i) = HomoFrame[6](1, 3) - HomoFrame[i](1, 3);
		Oi_On(2, i) = HomoFrame[6](2, 3) - HomoFrame[i](2, 3);
	}
	Eigen::Matrix<float, 3, 1 > Oi_On_Arr;
	Eigen::Matrix<float, 3, 1 > Zi_Arr;
	Eigen::Matrix<float, 3, 1 > Oi_On_Cross_Zi;
	for (int i = 0; i < 6; i++)
	{
		Oi_On_Arr(0, 0) = Oi_On(0, i);
		Oi_On_Arr(1, 0) = Oi_On(1, i);
		Oi_On_Arr(2, 0) = Oi_On(2, i);
		Zi_Arr(0, 0) = Zi(0, i);
		Zi_Arr(1, 0) = Zi(1, i);
		Zi_Arr(2, 0) = Zi(2, i);
		Oi_On_Cross_Zi = Zi_Arr.cross(Oi_On_Arr);
		Jac(0, i) = Oi_On_Cross_Zi(0, i);
		Jac(1, i) = Oi_On_Cross_Zi(1, i);
		Jac(2, i) = Oi_On_Cross_Zi(2, i);
		Jac(3, i) = Zi(0, i);
		Jac(4, i) = Zi(1, i);
		Jac(5, i) = Zi(2, i);
	}
}