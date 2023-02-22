#ifndef BLOOM_PHD_FILTER_GAUSSIANMIXTURE_HPP
#define BLOOM_PHD_FILTER_GAUSSIANMIXTURE_HPP

//#include "bloom_phd_filter.hpp"
#include "sensor_msgs/PointCloud.h"
#include "uri_soft_base/gaussian.hpp"
#include "Eigen/Core"
#include <Eigen/Dense>
#include <ostream>



namespace bloom_filter
{
	class gaussian_component: public uri_soft_base::Gaussian{

		friend class  phdfilter;

		friend class readfile;

		protected:
		
			double _w; //weight for each component
		
		public:
			~gaussian_component();

			gaussian_component(Eigen::Vector2d m, double weight=1.0);
			
			gaussian_component();/*default constructor*/

			/*constructor for point cloud, default weight is 1*/
			gaussian_component(const geometry_msgs::Point32& point,double weight=1.0);
			
			virtual void print ();/*print to the commandline*/
			
			double getweight();/*provide weight*/

			void setweight ( double value);/*set weight*/
 		
			/*get the gaussian density*/
	 		friend double get_gdensity (Eigen::Vector2d z,Eigen::Vector2d& m,Eigen::Matrix2d& c);

			
			/*operator overlaoding*/
			friend std::ostream& operator<<(std::ostream& os,gaussian_component& gc);
     
			/*operator overlaoding- print a vector*/
			friend std::ostream& operator<<(std::ostream& os,std::vector<gaussian_component>& gc);

			gaussian_component& operator=(const gaussian_component& second);

			geometry_msgs::Point32 get_geomtryPt(void);//direct convertion to ROS msg

			//gaussisn_component(gaussian_component&& ) noexcept;//move constructor
			friend 	void set_weight_nomaliz_n_prune(std::vector<gaussian_component>& out,std::vector<gaussian_component>& in, double& wsum, double pW );


	 };

	double get_gdensity (Eigen::Vector2d z,Eigen::Vector2d m,Eigen::Matrix2d c);

	void set_weight_nomaliz_n_prune(std::vector<gaussian_component>& out,std::vector<gaussian_component>& in, double& wsum, double pW );

	void set_normalize_w(std::vector<gaussian_component>& out,std::vector<gaussian_component>& in, double& wsum);


} // namespace bloom_filter


 
#endif
