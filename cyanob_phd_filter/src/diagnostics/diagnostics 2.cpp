#include "diagnostics.hpp"
using namespace std;
namespace bloom_filter{

	diagnostics::diagnostics (ros::NodeHandle n):_nh(n){
		_issysteminitiated=false;

		_nh.param<string>("folder_path",_folder_path,"~/catkin_ws/src/uri_soft_wip/cyanobloom/cyanob_phd_filter/results/");
		_nh.param<string>("init_filename",_init_filename,"coordinates.txt");
		_nh.param<int>("cycles",_cycles,5);//number of times filtering is perfomed
		if(_cycles==0) {
			cout<<"cycle number is invalid.....Exiting!!!"<<endl;
			exit(1);
		}

	    _pub = _nh.advertise<cyanob_phd_filter::diagnostics>("system_Diagnostics",5,true);

	    _sub = _nh.subscribe("system_Diagnostics",5,&diagnostics::Sub_diagnosticNode,this);
	    cout<<"filter will be performed "<<_cycles<<" times."<<endl;
	    std::cout<<"system diagnostics node is running. delayed 10 sec to initiate system"<<endl;
		_cycle_counter=0;

		//unique_ptr<readfile> log(new diagnosti(_folder_path+"log"+gettime()+".txt"));
		readfile::_filepath=_folder_path+"log"+gettime()+".txt";
		



		//system waits 10 sec till other nodes initiates
		//system then send a status request msg
		//req_stat_timer=_nh.createTimer(ros::Duration(10),&diagnostics::timerCallback,this,true);

	}

	void diagnostics::timerCallback(const ros::TimerEvent& event){
		cout<<"system diagnostics requested status from bloom publisher node"<<endl;
		cyanob_phd_filter::diagnostics req_status;
		req_status.bloom_pub="request_status";//requesting status from bloom points publisher
		_pub.publish(req_status);

	}

	void diagnostics::send_hold_msg(){
		msg.time_update="hold";
		_pub.publish(msg);
		//ros::spinOnce();
	}

	cyanob_phd_filter::diagnostics diagnostics::copy_msg (cyanob_phd_filter::diagnostics& m){

		cyanob_phd_filter::diagnostics copy = m;

	}

	void  diagnostics::Sub_diagnosticNode(const cyanob_phd_filter::diagnostics::ConstPtr& msg ){
		lastmsg=*msg;
		write2file(*msg);
		cout<<"========================"<<msg->header.seq<<"======================"<<endl;
		cout<< "time_update:"<<msg->time_update<<endl;
		cout<< "measurement_update:"<<msg->measurement_update<<endl;
		cout<< "simulation:"<<msg->simulation<<endl;
		cout<< "filename:"<<msg->filename<<endl;
		cout<< "bloom_pub:"<<msg->bloom_pub<<endl;
		cout<<"========================"<<msg->header.seq<<"======================"<<endl;

		if(_issysteminitiated==false){//waiting for all other node to be ready
			if(msg->time_update=="ready" && msg->measurement_update=="ready"){
				cout<<"system is initiating"<<endl;
				lastmsg.header.frame_id= "PHD filter status";
				lastmsg.filename=_init_filename;
				lastmsg.time_update="activate";
				_pub.publish(lastmsg);
				_issysteminitiated=true;
			}
		}

		if(msg->measurement_update=="activate"){
			_cycle_counter++;
			cout<<"cycle counter = "<<_cycle_counter<<endl;
		}
		if(_cycle_counter>_cycles){
			cout<<"number of cycles reached...exiting the program.............."<<endl;
			send_hold_msg();
			exit(1);
		}
	}

	void diagnostics::write2file(cyanob_phd_filter::diagnostics msg){
		if(readfile::_file.is_open()){
			_file<<"========================"<<msg.header.seq<<"======================"<<endl;
			_file<< "time_update:"<<msg.time_update<<endl;
			_file<< "measurement_update:"<<msg.measurement_update<<endl;
			_file<< "simulation:"<<msg.simulation<<endl;
			_file<< "filename:"<<msg.filename<<endl;
			_file<< "bloom_pub:"<<msg.bloom_pub<<endl;
			_file<<"========================"<<msg.header.seq<<"======================"<<endl<<endl;
		}

	}

	// void diagnostics::validate_file (){// virtual function from readfile class
	// 		readfile::_file.open(_filepath,std::fstream::out|std::fstream::app);//trunc -all previous content deleted. use app to append.
    //         std::cout<<"validating the file path - inside diagnostic class"<<std::endl;
	// 		cout<<_filepath<<endl;
	// 		if(_file.is_open()){
	// 			cout<<"file has opened succefully"<<endl;
	// 			cout<<_filepath<<endl; 
	// 			readfile::_file<<"log file for system diagnostics"<<endl;
	// 			readfile::_file<<"==============================="<<endl<<endl;
	// 		}
	// 		else{
	// 			std::cerr << "file opening error. Exiting..."<<std::endl;
	// 			throw 2;
	// 			std::exit(1);
	// 		}


    //        std::cout<<"File validated.."<<std::endl;
	
	
	// }
	string diagnostics::gettime(){
		/*getting system time*/
		time_t rawtime;
		struct tm * timeinfo;
		time (&rawtime);
  		timeinfo = localtime (&rawtime);
		string time1;
		time1=to_string(timeinfo->tm_year+1900)+to_string(timeinfo->tm_mon+1)+to_string(timeinfo->tm_mday)+to_string(timeinfo->tm_hour)+to_string(timeinfo->tm_min);
		cout<<"time now is "<<time1<<endl;
		return time1;
	}

	diagnostics::~diagnostics(){
		if(readfile::_file.is_open()){
			readfile::_file.close();
		}
	}


}

