#include "../bloom_phd_filter_gaussianMixture.hpp"
#include "time.h"
#include "diagnostics.hpp"

#ifndef SYSTEM_DIAGNOSTICS_PARALLEL_HPP
#define SYSTEM_DIAGNOSTICS_PARALLEL_HPP


using namespace std;
namespace bloom_filter{


        struct system_diagnostics_parallel : public diagnostics {

                system_diagnostics_parallel(ros::NodeHandle n);

                ros::NodeHandle _nh;

                void Sub_diagnosticNode(const cyanob_phd_filter::diagnostics::ConstPtr& msg );

                ~system_diagnostics_parallel();

                double delay;

        };


}


#endif