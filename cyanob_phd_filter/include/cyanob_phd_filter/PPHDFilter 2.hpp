#include "ros/ros.h"
#include "std_msgs/String.h"

#include <Eigen/Dense>
#include "cyanob_phd_filter/Particle.hpp"

/* ... */
//EIGEN_DEFINE_STL_VECTOR_SPECIALIZATION(eigen::Matrix2d)

#include<Eigen/StdVector>

#ifndef PPHDFILTER_HPP
#define PPHDFILTER_HPP



namespace cyanobloom{





class PPHD : public std::vector<Particle>
    {

	int unitW;
	
	public:
        // default constructor (creates an empty 2D gaussian mixture)
        PPHD();

        // copy constructor
        PPHD(const PPHD &other);

        friend std::ostream& operator<<(std::ostream& os, const PPHD& p);

};




class PPHDFilter{


	PPHD *prior;
	PPHD *nextprior;
	
	PPHD *post;
	PPHD *nextpost;
	
	PPHD prior1;
	PPHD prior2;
	
	PPHD post1;
	PPHD post2;

	public:
	
		PPHDFilter();
	
		void initialize(unsigned int np, GaussianMixture gm);

		void timeUpdate();

		void measurementUpdate();

        friend std::ostream& operator<<(std::ostream& os, const PPHDFilter& f);

};


}; // end namespace









#endif



