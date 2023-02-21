#ifndef SYSTEM_DIAGNOSTICS_PARALLEL_CPP
#define SYSTEM_DIAGNOSTICS_PARALLEL_CPP

#include "system_diagnostics_parallel.hpp"
using namespace std;
namespace bloom_filter{

	system_diagnostics_parallel::system_diagnostics_parallel (ros::NodeHandle n):_nh(n){
		_issysteminitiated=false;

		delay=15.0;

		// _nh.param<string>("folder_path",_folder_path,"~/catkin_ws/src/uri_soft_wip/cyanobloom/cyanob_phd_filter/results/");
		// _nh.param<string>("init_filename",_init_filename,"coordinates.txt");
		string _sysdiagTopicname;
        _nh.param<string>("diaganostic_topicName",_sysdiagTopicname,"system_Diagnostics");//system diagnostic topic name
       
		_nh.param<int>("cycles",_cycles,10);//number of times filtering is perfomed
		if(_cycles==0) {
			cout<<"SDP->cycle number is invalid.....Exiting!!!"<<endl;
			exit(1);
		}

	    _pub = _nh.advertise<cyanob_phd_filter::diagnostics>(_sysdiagTopicname,5,true);

	    _sub = _nh.subscribe(_sysdiagTopicname,5,&system_diagnostics_parallel::Sub_diagnosticNode,this);

	    cout<<"filter will be performed "<<_cycles<<" times."<<endl;
	    std::cout<<"system diagnostics node is running."<<endl;
		_cycle_counter=0;

		//unique_ptr<readfile> log(new diagnosti(_folder_path+"log"+gettime()+".txt"));
		//readfile::_filepath=_folder_path+"log"+gettime()+".txt";

	}

	void  system_diagnostics_parallel::Sub_diagnosticNode(const cyanob_phd_filter::diagnostics::ConstPtr& msg ){
		lastmsg=*msg;
		write2file(*msg);
		cout<<"========================"<<msg->header.seq<<"======================"<<endl;
		cout<< "time_update:"<<msg->time_update<<endl;
		cout<< "measurement_update:"<<msg->measurement_update<<endl;
		cout<< "simulation:"<<msg->simulation<<endl;
		cout<< "filename:"<<msg->filename<<endl;
		cout<< "bloom_pub:"<<msg->bloom_pub<<endl;
		cout<<"========================"<<msg->header.seq<<"======================"<<endl;


		//gui will have a button that initiate status request to bloom node
		//bloom node will then request status --> Time update which captured here
		//This is initiating step
		if(_issysteminitiated==false){
			if(msg->time_update=="request_status" ){
				cout<<"SDP->system is initiating"<<endl;
				lastmsg.header.frame_id= "PHD parallel filter status";
				//lastmsg.filename=_init_filename;
				lastmsg.time_update="activate";
				//lastmsg.bloom_pub="activate";
				_pub.publish(lastmsg);
				_issysteminitiated=true;
			}
		}

		if(msg->measurement_update=="stop" && msg->simulation=="droneatHome"){
			//stop is sent by simulation after drone completed the path
			//unlike diagnostics here we have to manage tu and mu since tu mu avoid communicating with sys manager
			_cycle_counter++;
			cout<<"SDP->completed drone trials = "<<_cycle_counter<<endl;
			cout<<"SDP->waiting "<< "1" <<"  seconds to Save MU File..."<<endl;
			ros::Duration(1).sleep();
			cout<<"SDP->activating TU..."<<endl;
			lastmsg.bloom_pub="activate";
			//bloom node will send tu->activate msg after receiving this
			lastmsg.measurement_update="ready";
			lastmsg.filename="completed drone trials = "+to_string(_cycle_counter);
			if(_cycle_counter==_cycles){
				cout<<"SDP->number of cycles reached...exiting the program.............."<<endl;
				send_hold_msg();
				exit(1);
			}
			_pub.publish(lastmsg);
		}




		if(msg->time_update=="activate"){
			//time update activated first, then measurement update started.
			//here we have to compensate time that takes measurement update to start and be ready
			cout<<"SDP->waiting "<< delay <<" seconds to finish time update..."<<endl;
			ros::Duration(delay).sleep();
			cout<<"SDP->activating new simulation..."<<endl;
			//inform simulation to start
			lastmsg.time_update="ready";
			lastmsg.measurement_update="running";
			lastmsg.simulation="activate";

			//if (_issysteminitiated==true) lastmsg.bloom_pub="activate";
			//in the start, during request status, bloom node reads the first file
			//after that we start bloom node same time with simulation

			_pub.publish(lastmsg);
		}



	}

	system_diagnostics_parallel::~system_diagnostics_parallel(){
		this->~diagnostics();
	}


}

int main (int argc, char *argv[]){

    std::cout<<"system diagnostics node is starting"<<endl;

    ros::init(argc,argv,"system_diagnostics_parallel");

    ros::NodeHandle n;

    //ros::AsyncSpinner spinner(0);// trying async multithreader

    //spinner.start();

    std::unique_ptr<bloom_filter::system_diagnostics_parallel> filter_diag (new bloom_filter::system_diagnostics_parallel(n));

    ros::spin();

    ros::waitForShutdown();

    return 0;

}


#endif
