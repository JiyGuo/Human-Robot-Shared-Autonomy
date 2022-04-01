
#include <ros/ros.h>
#include "gmm.h"
#include <gmm/GetConstraint.h>

WSC::Gmm *gmm_ptr ;

bool CoupleCallback(gmm::GetConstraint::Request  &req, gmm::GetConstraint::Response &res ){
    WSC::Data data = Eigen::VectorXd::Zero(req.data.size());
    for (size_t i = 0; i < req.data.size(); i++){
        data(i) = req.data[i];
    }

    //    WSC::Couple couple_term = WSC::CoupleTerm(data, *gmm_ptr);
    WSC::Couple couple_term = WSC::CoupleTermPF(data, *gmm_ptr);
//    std::cout<<" couple_term : " ;
//    WSC::PrintVec(couple_term);
    for (size_t i = 0; i < couple_term.size(); i++){
        res.couple_term.push_back(couple_term(i));
    }
    return true;
}


int main(int argc, char **argv){
	ros::init(argc, argv, "constriant_server");
	ros::NodeHandle n;
	ros::ServiceServer service = n.advertiseService("gmm/get_constriant_couple_term", CoupleCallback);

    // build the GMM model
    WSC::PriorVector  priors;
    WSC::MeanVector means;
    WSC::SigmaVector sigmas;
    const std::string xml_path = "/home/jiyguo/ur_ws/src/gmm/data/GMM_7.xml";

    // if( !WSC::ReadXML(xml_path,priors,means,sigmas)){
    //     ROS_ERROR( "Something wrong!" );
    //     return -1;
    // }

    // std::cout<<"priors: " <<std::endl;
    // for (size_t i = 0; i < priors.size(); i++)
    // {
    //     std::cout<<priors[i] << " , " ;
    // }
    // std::cout<<std::endl;

    // std::cout<<" means : " <<std::endl;
    // for (size_t i = 0; i < means.size(); i++)
    // {
    //     WSC::PrintVec(means[i]);
    // }

    // std::cout<<" sigmas : "<<std::endl ;
    // for (size_t i = 0; i < sigmas.size(); i++)
    // {
    //     WSC::PrintMat(sigmas[i]);
    // }
    
    gmm_ptr = new WSC::Gmm(xml_path);

    WSC::Data data(3);
    data << 0.52 , -0.22 , 1.34306 ;
    WSC::Couple couple_term = WSC::CoupleTermPF(data, *gmm_ptr);
    WSC::PrintVec(couple_term);

    ROS_INFO("WSC services now ready");
	ros::spin();
	return 0;
}
