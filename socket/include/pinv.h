#include <Eigen/Dense>
typedef Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> MatXf;
// ������α��
MatXf pinv(MatXf x);