#ifndef UNIT_TEST_HPP
#define UNIT_TEST_HPP

#include "bloom_phd_filter.hpp"
#include "bloom_phd_filter_readfile.hpp"
//#include <filesystem>



using namespace bloom_filter;


struct unit_test : public bloom_filter::readfile, public bloom_filter::phdfilter{

    unit_test(ros::NodeHandle _n);

    void sample_points(void);

    void test_extract_pts(void);

    void test_measurment_update(void);

    void test_multiple_measurment_update(void);

    void test_callback_usv(ros::NodeHandle _n);

    void test_writefile(void);

    geometry_msgs::Point32 get_geo(int,int,int);

    friend std::ostream& operator<< (std::ostream& out,geometry_msgs::Point32& p);

    friend class bloom_filter::readfile;

    void test_read_record2list(void);//gaussian component vector

    /* data */
};

#endif