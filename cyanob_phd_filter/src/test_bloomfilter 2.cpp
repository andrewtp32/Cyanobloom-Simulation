#include "bloom_phd_filter.hpp"

//using namespace cyanobloom;

int main(int argc, char** argv){

	// bloom_filter::gaussian_component t;
	// t.setweight(10);
	// cout<<"weight = "<<t.getweight()<<endl;
	// cout<<"checking operator "<<t<<endl;

	// bloom_filter::writefile testfile ("path");

	//bloom_filter::readfile name;
	//cin>>name;
	

	//std::cin >> outsidefile;

	// /home/thiw1ka/catkin_ws/src/uri_soft_wip/cyanobloom/cyanob_phd_filter/src/coordinates.txt

	ros::init(argc, argv, "test_bloom_filter_node");
	
	ros::NodeHandle nh;

	ros::AsyncSpinner spinner(0);// trying async multithreader

	//ros::spin(mt);

	spinner.start();

	bloom_filter::phdfilter filter = nh;

	ros::waitForShutdown();

	return 0;
}







