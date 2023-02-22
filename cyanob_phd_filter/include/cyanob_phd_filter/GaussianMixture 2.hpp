#include "ros/ros.h"
#include "std_msgs/String.h"

#include <Eigen/Dense>

#include <random>
#include <iostream>

#include "cyanob_phd_filter/GaussianComponent.hpp"

#ifndef GAUSSIANMIXTURE_HPP
#define GAUSSIANMIXTURE_HPP





namespace cyanobloom{

class GaussianMixture : public std::vector< GaussianComponent >
{
	int dim;
	std::vector<double> weightScale;
	double weightSum;
	
	double computeWeightSum();

	public:
	
		// default constructor (creates an empty 2D gaussian mixture)
        GaussianMixture();
	  
        // size only constructor
        GaussianMixture(int dimension);
		
		// copy constructor
		GaussianMixture(const GaussianMixture &other);

		// draw a sample
		void drawSample(std::vector<Eigen::VectorXd> &out, unsigned int numSamples);
		
        friend std::ostream& operator<<(std::ostream& os, const GaussianMixture& g);

        double getWeightSum();
    
};





}; // end namespace








#endif



