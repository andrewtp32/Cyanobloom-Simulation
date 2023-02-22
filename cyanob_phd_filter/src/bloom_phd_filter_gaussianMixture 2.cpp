//#include "bloom_phd_filter.hpp"
#include "bloom_phd_filter_gaussianMixture.hpp"

namespace bloom_filter{

    bloom_filter::gaussian_component::gaussian_component(Eigen::Vector2d m, double weight):_w(weight){
        this->setMean(m);
        //set_observationMatrix();
        //set_ObsNoiceCov();
        this->setCovariance(Eigen::Matrix2d::Identity()*0.25);
    }

    bloom_filter::gaussian_component::gaussian_component():_w(1){}

    /*in this constructor default weight is set to 1*/
    bloom_filter::gaussian_component::gaussian_component(const geometry_msgs::Point32& point,double weight){
        //std::cout<<"point2gaussuan = "<<point->x<<" , "<<point->y<<std::endl;
        gaussian_component(Eigen::Vector2d(point.x,point.y),weight);
    }


    double bloom_filter::gaussian_component::getweight(){
        return this->_w;
    }

    void bloom_filter::gaussian_component::print() {

        std::cout<<"Weight = "<<this->getweight()<<std::endl;
        //std::cout<<"mean"<<std::endl<<this->getMean()<<std::endl;
        //std::cout<<"Cov"<<std::endl<<this->getCovariance()<<std::endl;
        
    }

    void bloom_filter::gaussian_component::setweight( double  value){
        this->_w=value;
    }

    std::ostream& operator<< (std::ostream& os,gaussian_component& gc){
            gc.print();
        return os;
    }

    std::ostream& operator<< (std::ostream& os,std::vector<gaussian_component>& gc){
        for (gaussian_component _it:gc){
            os<<_it;
        }
        return os;
    }

    double get_gdensity (Eigen::Vector2d z,Eigen::Vector2d m,Eigen::Matrix2d c){

        Eigen::Vector2d error;
        //std::cout<<"z & m ="<<z<<std::endl<<m<< std::endl<<c<< std::endl;
        error = z-m;
        //std::cout<<"error ="<<error<< std::endl;
        double sum=-0.5*error.transpose()*c.inverse()*error;
        
        double sum2 = sqrt(4*M_PI*M_PI*abs(c.determinant())); //change the 2pi values
          //  std::cout<<"sum 1="<<sum<<", sum 2 = "<<sum2<<std::endl;
        //return exp(sum)/sum2;
        //std::cout<<"sum ="<<exp(sum)<<std::endl;
        return  exp(sum);
   }
    
    void set_weight_nomaliz_n_prune(std::vector<gaussian_component>& out,std::vector<gaussian_component>& in, double& wsum, double pW ){
        //pw - prune weight
        //wsum - normalizing weight
        for (gaussian_component _it:in){

            double _final_weight=_it._w/wsum;
            std::cout<<"weight = "<<_it._w<<"weight sum = "<<wsum<<", final weight = "<<_final_weight<<std::endl;

            if (_final_weight>pW){
                std::cout<<"inside weight = "<<_it._w<<"weight sum = "<<wsum<<", final weight = "<<_final_weight<<std::endl;
                _it.setweight(_final_weight);

                out.push_back(_it);

            }
        }
    }

    gaussian_component& gaussian_component::operator=(const gaussian_component &second){

        Gaussian::operator=(second);
        this->_w=second._w;

        return *this;

    }


    geometry_msgs::Point32 gaussian_component::get_geomtryPt(void){
        geometry_msgs::Point32 temp;
        temp.x=mean.x();
        temp.y=mean.y();
        temp.z=_w;
        return temp;
    }

    gaussian_component::~gaussian_component(){
        //std::cout<<"Gaussian component destructor"<<std::endl;

    }




}

