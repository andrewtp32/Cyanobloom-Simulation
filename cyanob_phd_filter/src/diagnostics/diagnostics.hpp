#include "../bloom_phd_filter_gaussianMixture.hpp"
#include "../bloom_phd_filter_readfile.hpp"
#include "time.h"

#ifndef DIAGNOSTICS_HPP
#define DIAGNOSTICS_HPP

#include "cyanob_phd_filter/diagnostics.h"
using namespace std;
namespace bloom_filter{


        struct diagnostics: public readfile{

                diagnostics(){};

                diagnostics(ros::NodeHandle n);

                ros::NodeHandle _nh;

                ros::Publisher _pub;

                ros::Subscriber _sub;

                void Sub_diagnosticNode(const cyanob_phd_filter::diagnostics::ConstPtr& msg );

                cyanob_phd_filter::diagnostics msg;

                void send_hold_msg();

                cyanob_phd_filter::diagnostics copy_msg (cyanob_phd_filter::diagnostics& m);

                int _cycles, _cycle_counter;

                cyanob_phd_filter::diagnostics lastmsg;

                string _init_filename, _folder_path;//file name. inside the folder.

                bool _issysteminitiated;

                ros::Timer req_stat_timer;//timer to introduce initiating delay

                void timerCallback(const ros::TimerEvent& event);

                void write2file(cyanob_phd_filter::diagnostics msg);

                //void validate_file ();// virtual function from readfile class
                
                string gettime();

                ~diagnostics();



        };


}


#endif