


#include "cyanob_phd_filter/PPHDFilter.hpp"




namespace cyanobloom{







// Methods oif class GaussianMixture
PPHD::PPHD()
{
}


// copy constructor
PPHD::PPHD(const PPHD &other)
{
    for (int i=0; i<other.size(); i++){
        this->push_back(other[i]);
    }
}

std::ostream& operator<<(std::ostream& os, const PPHD& g)
{
    std::cout << "# of particles: "<< g.size() << std::endl;
    for (int i=0; i<g.size(); i++){
        std::cout << g[i] << std::endl;
    }
  return os;
}































PPHDFilter::PPHDFilter()
{
	prior = &prior1;
	post = &post1;
	nextprior = &prior2;
	nextpost = &post2;
}




void PPHDFilter::initialize(unsigned int np, GaussianMixture gm)
{
	std::vector <Eigen::VectorXd> pa;
	gm.drawSample(pa, np);
	nextprior->clear();
	nextprior->reserve(np);

    double ws = gm.getWeightSum();

	for (int i = 0; i<np; i++)
	{
		nextprior->push_back(Particle(pa[i], ws/(double)np));
	}
	PPHD *temp = prior;
	prior = nextprior;
	nextprior = temp;
}




void timeUpdate()
{
	//PPHD *temp = prior;
	//prior = nextprior;
	//nextprior = temp;
}

void measurementUpdate()
{

}


std::ostream& operator<<(std::ostream& os, const PPHDFilter& f)
{
    std::cout << "prior: "<< *(f.prior) << std::endl;
    std::cout << "posterior: "<< *(f.post) << std::endl;
  return os;
}





}; // end namespace





