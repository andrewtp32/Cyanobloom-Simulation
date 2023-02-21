


#include "cyanob_phd_filter/Particle.hpp"





namespace cyanobloom{



Particle::Particle() : Particle(Eigen::VectorXd(2),0.0)
{
}

Particle::Particle(Eigen::VectorXd const& m, double w)
{
	mean = m;
	weight = w;
}



Particle::Particle(const Particle &other) : Particle(other.mean, other.weight)
{
}


Particle& Particle::operator= (const Particle &other)
{
  // do the copy
	if(this != &other)
  {
  	mean = other.mean;
  	weight = other.weight;
  }
  return *this;
}


std::ostream& operator<<(std::ostream& os, const Particle& g)
{
    std::cout << " w: " << g.weight << " mean: " <<  g.mean << std::endl;
    return os;
}

}; // end namespace























