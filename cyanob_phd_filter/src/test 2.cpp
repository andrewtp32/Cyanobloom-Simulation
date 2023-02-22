
#include "cyanob_phd_filter/PPHDFilter.hpp"

using namespace cyanobloom;

int main(void){

	int d = 2;
	Eigen::VectorXd m1(d),m2(d),m3(d);
	m1 << 20, 30;
	m2 << 40, 0;
	m3 << 0, -10;
	Eigen::MatrixXd r1(d,d), r2(d,d), r3(d,d);
	r1 << 1, .5, .5, 1;
	r2 << 2, 1.5, 1.5, 2;
	r3 << 3.5, 1.5, 1.5, 1;
		      
	GaussianComponent c1(Gaussian(m1, r1), 0.5);
	GaussianComponent c2(Gaussian(m2, r2), 0.3);
	GaussianComponent c3(Gaussian(m3, r3), 0.9);

	GaussianMixture gm;
	gm.push_back(c1);
	gm.push_back(c2);
	gm.push_back(c3);
	
  std::cout << gm << std::endl;

	std::vector<Eigen::VectorXd> samples;
	gm.drawSample(samples, 500);


	PPHDFilter pphdf;

	pphdf.initialize(10, gm);



	std::cout << pphdf << std::endl;

//	std::cout << s.drawSample() << std::endl;

	return 0;
}







