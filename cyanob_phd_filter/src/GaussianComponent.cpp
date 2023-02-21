


#include "cyanob_phd_filter/GaussianComponent.hpp"






namespace cyanobloom {


	GaussianComponent::GaussianComponent(Gaussian g, double w)
	{
		first = g;
		second = w;
	}


	std::ostream& operator<<(std::ostream& os, const GaussianComponent& g)
	{
			std::cout << "weight: " << g.weight() << std::endl << g.gaussian();
	    return os;
	}

}; // end namespace






























