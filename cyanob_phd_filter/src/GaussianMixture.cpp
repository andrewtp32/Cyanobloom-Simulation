


#include "cyanob_phd_filter/GaussianMixture.hpp"





namespace cyanobloom{



double GaussianMixture::computeWeightSum()
{
	weightScale.resize(this->size());
	
	weightScale[0] = (*this)[0].weight();
	for (int i=1; i<this->size(); i++){
		weightScale[i] = weightScale[i-1] + (*this)[i].weight();
	}
	weightSum = weightScale[this->size()-1];
	for (int i=0; i<this->size(); i++){
		weightScale[i] = weightScale[i]/weightScale[this->size()-1];
//		std::cout << weightScale[i] << std::endl;
	}
	
	return weightSum;
}



double GaussianMixture::getWeightSum()
{
	weightSum = 0.0;
	for (int i=0; i<this->size(); i++){
		weightSum += (*this)[i].weight();
	}
	return weightSum;
}




// Methods if class GaussianMixture
GaussianMixture::GaussianMixture():GaussianMixture(2)
{
}

// size only constructor (zero mean, identity matrix covariance)
GaussianMixture::GaussianMixture(int dimension)
{
	dim = dimension;
	this->clear();
}

// copy constructor
GaussianMixture::GaussianMixture(const GaussianMixture &other):GaussianMixture(other.dim)
{
	for (int i=0; i<other.size(); i++){
		this->push_back(other[i]);
	}
}

// draw a sample
void GaussianMixture::drawSample(std::vector<Eigen::VectorXd> &output, unsigned int numSamples = 1)
{
	srand (time(NULL));	
	computeWeightSum();
	
	output.resize(numSamples);
	
		int c0=0, c1=0, c2=0;

	for (int j=0; j < numSamples; j++)
	{
		int extraction = rand() % 100;
		double extr = ((double)extraction)/100.0;
//		std::cout << extr << std::endl;
		int i;
		for (i=0; i<this->size(); i++){
//			std::cout << i << " " << extr << " "<< weightScale[i] << std::endl;
			if (extr < weightScale[i]) break;
		}
		output[j] = (*this)[i].gaussian().drawSample();
		
		//std::cout << (*this)[i] << std::endl;
		
//		std::cout << i << " "<< output[j].transpose() << std::endl;
		if (i==0) c0++;
		if (i==1) c1++;
		if (i==2) c2++;
	}
	//	std::cout << c0 << " " << c1 << " " << c2 << " " << std::endl;
}

std::ostream& operator<<(std::ostream& os, const GaussianMixture& g)
{
	std::cout << "# of components: "<< g.size() << std::endl;
	for (int i=0; i<g.size(); i++){
		std::cout << g[i] << std::endl;
	}
  return os;
}







}; // end namespace























