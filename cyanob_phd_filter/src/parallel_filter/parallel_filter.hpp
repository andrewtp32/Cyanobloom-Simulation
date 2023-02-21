#include "../bloom_phd_filter.hpp"
#ifndef PARALLEL_FILTER_HPP
#define PARALLEL_FILTER_HPP
#include </usr/include/python2.7/pyconfig.h>
//export CPLUS_INCLUDE_PATH="$CPLUS_INCLUDE_PATH:/usr/include/python2.7"
#include <boost/python.hpp>
#include "../diagnostics/diagnostics.hpp"

namespace bloom_filter{

    struct parallel_filter: public phdfilter{

        parallel_filter(ros::NodeHandle _n);

        friend diagnostics;
        double _noise,_pruneWeight, _AllmeasSize;
        int _f,_ftheta,_t;
        string _fname,_folderpath,_pythonFileName,_pythonclassName;
        string _TuFilepath;
        //std::string _submtn,_pubtn;
        PyObject* pmodule;
        shared_ptr<phdfilter> phd_ptr;

        ros::NodeHandle _nh;

        bool _measurementupdateactivated;

        std::vector<gaussian_component> _Allmesurements;

        ros::Subscriber _sub_SDiagnostics;
        //ros::Publisher _pub;        

        void cb_systemD(const cyanob_phd_filter::diagnostics::ConstPtr& msg);

        //virtual function from the measurement update
        void callback_sys_diag(const cyanob_phd_filter::diagnostics::ConstPtr& msg);

        void Sub_diagnosticNode(const cyanob_phd_filter::diagnostics::ConstPtr& msg){};
 
        void print_list(std::vector<gaussian_component>&);

        void callback_camera_meas2(const sensor_msgs::PointCloud::ConstPtr& msg);

        string time_update(void);

        ~parallel_filter(void);


        
    };




}

#endif