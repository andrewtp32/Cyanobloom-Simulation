#include "ros/ros.h"
#include "std_msgs/String.h"

#include <Eigen/Dense>

#include <random>
#include <iostream>

#include "cyanob_phd_filter/GaussianMixture.hpp"

#ifndef PARTICLE_HPP
#define PARTICLE_HPP





namespace cyanobloom{





class Particle{

	public:
	
        Eigen::VectorXd mean;

		double weight;
		
        Particle();
		
        Particle(Eigen::VectorXd const& m, double w);


        Particle(const Particle &other);
		
		Particle& operator= (const Particle &other);

        friend std::ostream& operator<<(std::ostream& os, const Particle& p);

};



}; // end namespace








#endif



