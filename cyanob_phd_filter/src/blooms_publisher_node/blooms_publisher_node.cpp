#include "bloom_locations_publisher.hpp"

//using namespace cyanobloom;

int main(int argc, char** argv){

	ros::init(argc, argv, "gazebo_blooms_publisher_node");
	
	ros::NodeHandle nh; //~ means using namespace given in the launch file

	ros::AsyncSpinner spinner(0);// trying async multithreader

	//ros::spin(mt);

	spinner.start();

	string _topic_name,_fpath;

	nh.param<string>("readfile/topic_name",_topic_name,"particle_list_for_gazebo");

	//change the name "thivanka" below according to your username if launch file not used
	nh.param<string>("readfile/file_path",_fpath,"/home/thivanka/catkin_ws/src/uri_soft_wip/cyanobloom/cyanob_phd_filter/src/coordinates.txt");

	//bloom_filter::blooms_creator bloom_publiser(nh,_topic_name,_fpath,1);
	bloom_filter::blooms_creator bloom_publiser(nh);

	ros::waitForShutdown();

	return 0;
}

/*sample file path*/
/* 
/home/thiw1ka/catkin_ws/src/uri_soft_wip/cyanobloom/cyanob_phd_filter/src/coordinates.txt
*/
