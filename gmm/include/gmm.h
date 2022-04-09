
/**
  * \author Jiyang Guo
  */

#ifndef GMM_H_
#define GMM_H_

#include <cmath>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <vector>
#include <iostream>
#include "tinyxml2.h"
#include "string"
#include "vector"
#include <sstream>

namespace WSC{

  typedef Eigen::VectorXd Data;
  typedef Eigen::VectorXd Couple;
  typedef Eigen::VectorXd Gradiant;
  typedef Eigen::VectorXd Mean;
  typedef Eigen::MatrixXd Sigma;
  typedef std::vector<double> PriorVector;
  typedef std::vector<Mean> MeanVector;
  typedef std::vector<Sigma> SigmaVector;
  
  class Gmm
  {
  private:

    const double PSI = 0.000000000001;

    inline double GaussPDF(const Data& data, const Mean& mean, const Sigma& sigma) const ;

    int nb_cmpnts_;
    PriorVector  prior_vec_;
    MeanVector mean_vec_;
    SigmaVector sigma_vec_;

  public:
    Gmm(const PriorVector& priors, const MeanVector& means, const SigmaVector& sigmas );
    Gmm(const std::string& file_name );
    double CalPDF( const Data& data ) const;
    Gradiant CalGradiant( const Data& data  ) const;
    ~Gmm();
  };

  void PrintVec(const Eigen::VectorXd& vec) ;
  void PrintMat(const Eigen::MatrixXd& mat) ; 

  const double LB = 0.03;
  const double UB = 0.008;
  const double ALPHA = 80;
  const double MAX_APT = 10;

  Couple CoupleTerm(const Data& data, const Gmm& gmm);
  Couple CoupleTermPF(const Data& data, const Gmm& gmm);
  bool ReadXML(const char* file_name  , WSC::PriorVector&  priors, WSC::MeanVector& means, WSC::SigmaVector& sigmas);

}

#endif /* GMM_H_ */
