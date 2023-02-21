#ifndef BLOOM_PHD_FILTER_HPP
#define BLOOM_PHD_FILTER_HPP

#include <ros/ros.h>
#include "Eigen/Core"
#include <Eigen/Dense>

#include "bloom_phd_filter_param.hpp"
#include "bloom_phd_filter_gaussianMixture.hpp"
#include "bloom_phd_filter_readfile.hpp"
#include "diagnostics/diagnostics.hpp"
//#include "uri_soft_base/gaussian.hpp"
//#include "parallel_filter/parallel_filter.hpp"

#include "sensor_msgs/PointCloud.h"
#include "geometry_msgs/Pose.h"
#include "sensor_msgs/Temperature.h"
#include "cyanob_phd_filter/diagnostics.h"




namespace bloom_filter{

    class phdfilter{

       //friend parallel_filter;

     public:
        ros::NodeHandle _n;
        
        //ros::Timer _timer1, _timer2;

        ros::Subscriber _sub,_sub_dronepose, _sub_USV_mu, _sub_diagnos;

        ros::Publisher _pub, _pub_diagnos;

        std::string _submtn,_pubtn, _sub_dpose, _USV_topicname;

        std::string _fpath;//file path

        double _Pd;//probaility of detection

        double _pruneW;//prune weight

        geometry_msgs::Point32 _imgcornerpts [4];//four corners of the image

        //msg to publish in filter results topic
        sensor_msgs::PointCloud _msg_filter_results;

        //gaussian_component mesurements container;
        std::vector<gaussian_component> _mesurements;

        //belief container, all estimates
        std::vector<gaussian_component> _belief;

        /*subcriber to read from camera measurement topic*/
        void callback_camera_meas(const sensor_msgs::PointCloud::ConstPtr& msg);

        void callback_dronepose(const geometry_msgs::Pose::ConstPtr& msg);

        void callback_usv_meas(const sensor_msgs::Temperature::ConstPtr& msg);

        virtual void callback_sys_diag(const cyanob_phd_filter::diagnostics::ConstPtr& msg);

        /*measurement updated function*/
        /*update uses given list of prior points*/
        void measurement_update (std::vector<gaussian_component>&);

        //readfile* _initial_points_file;
        std::shared_ptr<readfile> _initial_points_file;

        //extract points relevant to the image
        void extract_relvant_points (std::vector<gaussian_component>&);

        double area_triang (geometry_msgs::Point32& corner1,geometry_msgs::Point32& corner2, double x3, double y3 );

        double get_total_area(gaussian_component&);//getting total area given 5 points

        void publish (void);//publish the results

        virtual void print_list(std::vector<gaussian_component>&);//print points inside the vector

        bool _isDroneAltitudesound;//is drone alt is correct?

        bool _isMUrequested; //is measurement update is requested.

        void get_points_insidecircle (geometry_msgs::Point32& center, double rad);
   
        public:
            phdfilter();

            /*Constructor for phd filter*/
            phdfilter(ros::NodeHandle n);

            void set_Pd (double p=0.6);//set probaility of detection.default value is 1

            void set_prune_weight (double pw=0.001);

            ~phdfilter();//destructor for the class

    };






}
#endif