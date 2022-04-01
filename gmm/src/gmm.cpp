#include "gmm.h"

namespace WSC{
    using tinyxml2::XMLDocument ;
    using tinyxml2::XMLError ;
    using tinyxml2::XMLElement ;
    using std::cout;
    using std::endl;
    using std::cerr;
    using std::vector;
    using std::string;
    using std::istringstream;

     /**
     * @brief constructed function 1
     *
     * @input:
     * @param priors      priors(saclar) vector of gmm
     * @param means    means(vector) vector of gmm
     * @param sigmas   sigmas(matrix)  vector of gmm
     *
     */
    Gmm::Gmm(const PriorVector& priors, const MeanVector& means, const SigmaVector& sigmas ){
        nb_cmpnts_ = means.size();
        prior_vec_ = priors;
        mean_vec_ = means;
        sigma_vec_ = sigmas;
    }

    /** 
     * @brief constructed function 2
     * 
     * @input:
     * @param file_name      xml file path of gmm
     *
     */
    Gmm::Gmm( const std::string& file_name ){
        try {
            bool has_read = ReadXML(file_name.c_str() ,prior_vec_, mean_vec_ , sigma_vec_ ) ;
            if ( !has_read)
                throw string("Something wrong ! "); 
        }
        catch (string str) {
            cerr << str << endl;
        }
        nb_cmpnts_ = mean_vec_.size();
    }

    /** 
     * @brief calculate 1 dimention GaussPDF
     * 
     * @input:
     * @param data      data needed calculated
     * @param mean      mean of gaussian model
     * @param sigma      sigma of gaussian model
     * 
     * @return GaussPDF value  (1 dimention) 
     */
    inline double Gmm::GaussPDF(const Data& data, const Mean& mean, const Sigma& sigma) const  {
        return 1.0 / sqrt( pow(2*M_PI,data.size())*( sigma.determinant() + PSI ) )  * exp( -0.5 * ( data - mean ).transpose() * sigma.inverse() * ( data - mean )) ;
    }

    /** 
     * @brief calculate PDF of GMM 
     *  
     * @input:
     * @param data      data needed calculated
     * 
     * @return PDF value  of GMM 
     */
    double Gmm::CalPDF(const Data& data ) const {
        double prob = 0;
        for (size_t i = 0; i < nb_cmpnts_; i++)
        {
//            cout<<"GaussPDF: "<<GaussPDF(data,mean_vec_[i],sigma_vec_[i])<<endl;

            prob += prior_vec_[i] * GaussPDF(data,mean_vec_[i],sigma_vec_[i]);
//            cout<<"prior_vec_: "<<prior_vec_[i]<<endl;
//            cout<<"mean_vec_: "<<endl;
//            PrintVec(mean_vec_[i]);
//            cout<<"sigma_vec_: "<<endl;
//            PrintMat(sigma_vec_[i]);
        }

        return prob;
    }

    /** 
     * @brief calculate Gradiant
     * 
     * @input:
     * @param data      data needed calculated
     * 
     * @return Gradiant vector of GMM
     */
    Gradiant Gmm::CalGradiant( const Data& data  ) const {
        Gradiant gradiant = Eigen::VectorXd::Zero(data.rows() , data.cols() );
        for (size_t i = 0; i < nb_cmpnts_; i++){
            gradiant += sigma_vec_[i].inverse() * (data - mean_vec_[i]) * prior_vec_[i] * GaussPDF(data, mean_vec_[i], sigma_vec_[i]);
        } 
        return gradiant;
    }

    /** 
     * @brief destructed function
     */
    Gmm::~Gmm(){}

    /** 
     * @brief Print  Eigen Vector
     *
     * @input:
     * @param vec: vector  needed to print
     * 
     */
    void PrintVec(const Eigen::VectorXd& vec) {
        for (size_t i = 0; i < vec.size(); i++){
            std::cout<< vec[i] ;
            if( i < vec.size()-1 )   std::cout<<" , ";
        }
        std::cout<<std::endl;
    }

    /** 
     * @brief    Print  Eigen Matrix: 
     * 
     * @input:
     * @param mat: matrix need to be print
     * 
    */
    void PrintMat(const Eigen::MatrixXd& mat) {
        for (size_t i = 0; i < mat.rows(); i++)
        {
             for (size_t j = 0; j < mat.cols(); j++)
            {
                std::cout<< mat(i,j) ;
                if( j < mat.cols()-1 )   std::cout<<" , ";
            }
            std::cout<<std::endl;
        }        
    }

    /** 
     * @brief calculate Sigmoid value 
     * 
     * @input:
     * @param prob      probability
     * 
     * @return probability value mapped by Sigmoid function
     */
    static inline double Sigmoid(const double& prob ){    
        double bias = (LB - UB) / 2 + UB;
        double sigmoid = 1 - 1/(1+exp(-ALPHA*( prob - bias )));
        return sigmoid;
    }

    /**
     * @brief calculate Potential Field value
     *
     * @input:
     * @param prob      probability
     *
     * @return probability value mapped by Potential Field function
     */
    static inline double PF( const double& prob ){
//        cout<<"prob: "<< prob << endl;
        double eta = 0.0005;
        double rol = prob - UB;
        double rol_0 = LB - UB;
        if (rol > rol_0) return 0;
        return eta*(1.0/rol - 1/rol_0) / pow(rol,2);
    }

    /** 
     * @brief calculate Amplitude value to constraint
     * @param prob      probability
     * 
     * @return constraint's Amplitude 
     */
    static double Amplitude( const double& prob ){
        return MAX_APT * Sigmoid( prob ) ;
    }

    /** 
     * @brief calculate Couple Term  to constraint
     * @input:
     * @param data      data needed calculated
     * @param gmm      gmm class
     * 
     * @return constraint's Couple Term (vector)
     */
    Couple CoupleTerm(const Data& data, const Gmm& gmm){
        double amplitude = Amplitude( gmm.CalPDF(data) );
        Couple couple_term = Eigen::VectorXd::Zero(data.size());
        Gradiant gradiant = gmm.CalGradiant(data);
        couple_term = -amplitude /  gradiant.norm() * gradiant;
        return couple_term;
    }

    /**
     * @brief calculate Couple Term  to constraint
     * @input:
     * @param data      data needed calculated
     * @param gmm      gmm class
     *
     * @return constraint's Couple Term (vector)
     */
    Couple CoupleTermPF(const Data& data, const Gmm& gmm){
        double amplitude = PF( gmm.CalPDF(data) );
//        cout<<"amplitude: "<< amplitude << endl;
        Couple couple_term = Eigen::VectorXd::Zero(data.size());
        Gradiant gradiant = gmm.CalGradiant(data);
//        cout<<"gradiant: ";
//        PrintVec(gradiant);
        couple_term = -amplitude / gradiant.norm() * gradiant;
        return couple_term;
    }

    /** 
     * @brief Split String
     * 
     * @input:
     * @param str      full string
     * @output:
     * @param strs     output :  saved string vector after split
     * 
     * @return whether str is empty 
     */
    static bool SplitStr(std::istringstream& str , vector<string>& strs){
        string tmp;
        int i = 0;
        while (str >> tmp) {
            strs.push_back(tmp);
            i++;
        }
        return i != 0 ;
    }

    /** 
     * @brief Read XML file to gmm data
     *
     * @param[in] file_name:xml path and file name
     * @param[out] priors:priors(saclar) vector of gmm
     * @param[out] means:means(vector) vector of gmm
     * @param[out] sigmas:sigmas(matrix)  vector of gmm
     * 
     * @return whether reading is successful
     */
    bool ReadXML(const char* file_name , WSC::PriorVector&  priors, WSC::MeanVector& means, WSC::SigmaVector& sigmas){
        XMLDocument doc;
        XMLError state = doc.LoadFile(file_name);
        if (state!=tinyxml2::XML_SUCCESS)
        {
            std::cerr << "Can not open this file" << endl;
            return false;
        }

        XMLElement* xml_root = doc.RootElement();

        // get number of guass kernal
        XMLElement* element = xml_root->FirstChildElement("nbKernal");
        string nbKernal_str = element->GetText();
        int nbKernal = stoi(nbKernal_str);

        // get log likelyhood of gmm
        element = xml_root->FirstChildElement("loglik");
        string loglik_str = element->GetText();
        float loglik = stof(loglik_str);

        // data process
        istringstream idata_str;
        vector<string> data_strs;
        string data_str;

        // get Priors of gmm
        element = xml_root->FirstChildElement("Priors");
        data_str = element->GetText();
        idata_str.str(data_str);
        priors.resize(nbKernal);
        if(!SplitStr( idata_str , data_strs)) {
            cerr<<"Prior data is empty ! "<<endl;
            return false;
        }
        if(data_strs.size()!=nbKernal){
            cerr<<"Prior's size is wrong ! "<<endl;
            return false;
        }
        for (size_t i = 0; i < nbKernal; i++)
        {
            priors[i] = std::stod(data_strs[i]);
        }
        data_strs.clear();
        idata_str.clear();

        // get Means of gmm
        element = xml_root->FirstChildElement("Means");
        string size_str = element->Attribute("size");
        idata_str.str(size_str);
        SplitStr( idata_str , data_strs);
        // get dimention of gaussian and check the number of kernal
        int dim = std::stoi(data_strs[0]);
        int nb = std::stoi(data_strs[1]);
        if( nb!=nbKernal ){
            cerr<<"Means's size is wrong ! "<<endl;
            return false;
        }
        data_strs.clear();
        idata_str.clear();
        // read data
        data_str = element->GetText();
        idata_str.str(data_str);
        means.resize(nb);
        if(!SplitStr( idata_str , data_strs)) {
            cerr<<"Means data is empty ! "<<endl ;
            return false;
        }
        for (size_t i = 0; i < nbKernal; i++)
        {
            WSC::Mean mean(dim);
            for (size_t j = 0; j < dim; j++)
            {
                mean(j) =  std::stod(data_strs[ dim * i + j ]);
            }
            means[i] = mean;
        }
        size_str.clear();
        idata_str.clear();
        data_strs.clear();

         // get Sigmas of gmm
        element = xml_root->FirstChildElement("Sigmas");
        // check the dimention of gaussian number of kernal
        size_str = element->Attribute("size");
        idata_str.str(size_str);
        SplitStr( idata_str , data_strs);
        int dim2 = std::stoi(data_strs[0]);
        nb = std::stoi(data_strs[2]);
        if( nb!=nbKernal || dim2!= dim ){
            cerr<<"Sigmas's size is wrong !     or   dimention is not equal to the Means ! "<<endl ;
            return false;
        }
        data_strs.clear();
        idata_str.clear();
        // read data
        data_str = element->GetText();
        idata_str.str(data_str);
        sigmas.resize(nbKernal);
        if(!SplitStr( idata_str , data_strs)) {
            cerr<<"Means data is empty ! "<<endl ;
            return false;
        }
        for (size_t i = 0; i < nbKernal; i++)
        {
            WSC::Sigma sigma(dim,dim);
            int base = dim * dim * i;
            for (size_t j = 0; j < dim; j++)
            {
                for (size_t k = 0; k < dim; k++)
                {
                    sigma(j,k) = std::stod(data_strs[ base + dim * j + k ]);
                }
            }
            sigmas[i] = sigma;
        }
        return true;
    }

};
