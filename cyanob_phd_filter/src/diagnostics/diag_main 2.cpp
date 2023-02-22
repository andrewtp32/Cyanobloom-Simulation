#include "diagnostics.hpp"
using namespace std;


    int main(int argc, char** argv){

        std::cout<<"system diagnostics node is starting"<<endl;

        ros::init(argc,argv,"Filter_diagnostics");

        ros::NodeHandle n;

        //ros::AsyncSpinner spinner(0);// trying async multithreader

        //spinner.start();

        std::unique_ptr<bloom_filter::diagnostics> filter_diag (new bloom_filter::diagnostics(n));

        ros::spin();

        ros::waitForShutdown();

        return 0;

    }
