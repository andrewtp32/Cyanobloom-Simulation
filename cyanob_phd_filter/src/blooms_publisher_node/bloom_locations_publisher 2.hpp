#ifndef BLOOM_LOCATIONS_PUBLISHER_HPP
#define BLOOM_LOCATIONS_PUBLISHER_HPP

#include <iostream>
#include <fstream>
#include <istream>
//#include <filesystem>
#include "../bloom_phd_filter.hpp"

namespace bloom_filter{

    class blooms_creator {

        ros::NodeHandle _npl;

        ros::Publisher _pub, _pub_diagnostic;

        ros::Subscriber _sub_diagnostics;

        std::vector<gaussian_component> _component_list;

        readfile* _outsidefile;

        sensor_msgs::PointCloud _pc_msg;

        std::string _topicname;

        sensor_msgs::PointCloud get_points();
        
        ros::Timer _timer;

        blooms_creator* bc_ptr; //pointer to bloom creator class

        string _bloomloc_fpath, _init_filename; //param that contains folder paths

        int _cycles, _fcounter;

        void publish_to_topic(const ros::TimerEvent&);//const ros::TimerEvent& f

        void callback_diagnostics(const cyanob_phd_filter::diagnostics::ConstPtr& msg);

    public:

        blooms_creator(ros::NodeHandle n,string _fname,string _foldPath,string topname );
        //will create a timer that publish at given rate, points inside _fname;
        //publish at 1hz

        blooms_creator(ros::NodeHandle n,std::string _topic,std::string _fpath, double rate );
        //file path should provide in the launch file

        blooms_creator(ros::NodeHandle _n);    
        /*this constructor is to publish several list of blooms when needed*/

        ~blooms_creator();

    };
}

    

#endif
