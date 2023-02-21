#include "ros/ros.h"
#include "std_msgs/String.h"

#include <Eigen/Dense>

#include <random>
#include <iostream>
#include "uri_soft_base/gaussian.hpp"


#ifndef GAUSSIANCOMPONENT_HPP
#define GAUSSIANCOMPONENT_HPP


using namespace uri_soft_base;

namespace cyanobloom {


class GaussianComponent : public std::pair<uri_soft_base::Gaussian, double>
{

	public:
	
		GaussianComponent(uri_soft_base::Gaussian, double);
	
        friend std::ostream& operator<<(std::ostream& os, const GaussianComponent& g);
	
        inline uri_soft_base::Gaussian gaussian() const {
            return this->first;
        }

        inline double weight() const {
            return this->second;
        }
};



}; // end namespace









#endif



