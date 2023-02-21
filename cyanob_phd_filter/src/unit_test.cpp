#include "unit_test.hpp"

//purpose of this class is to test the functions in phd filter
unit_test::unit_test(ros::NodeHandle _n):bloom_filter::phdfilter(_n){

    cout<<"____unit test node started______"<<endl;


}

geometry_msgs::Point32 unit_test::get_geo(int x,int y,int z=0){
    geometry_msgs::Point32 temp;
    temp.x=x;
    temp.y=y;
    temp.z=z;
    return temp;
}

std::ostream &operator<< (std::ostream& out,geometry_msgs::Point32& p){
    out<<"x = "<<p.x<<", y = "<<p.y<<endl;
    return out;

}

void unit_test::sample_points(void){
    this->_imgcornerpts[0]=(get_geo(-10,10)) ;
    this->_imgcornerpts[1]=(get_geo(10,10));
    this->_imgcornerpts[3]=(get_geo(10,-10));
    this->_imgcornerpts[2]=(get_geo(-10,-10));


    cout<<"boundry points"<<endl;
    for (int i=0;i<4;i++){
        cout<<_imgcornerpts[i];
    }

    this->_belief.clear();
    this->_belief.push_back(gaussian_component(Eigen::Vector2d(-100,-10),1.0));
    this->_belief.push_back(gaussian_component(Eigen::Vector2d(-1,-1),1.0));
    this->_belief.push_back(gaussian_component(Eigen::Vector2d(-3,-3),1));
    this->_belief.push_back(gaussian_component(Eigen::Vector2d(-11,-11),1));
    this->_belief.push_back(gaussian_component(Eigen::Vector2d(-1,-10),1));
    this->_belief.push_back(gaussian_component(Eigen::Vector2d(5,5),1));


    cout<<"points"<<endl<<this->_belief;

    this->_mesurements.clear();
    this->_mesurements.push_back(Eigen::Vector2d(-2,-2));
    this->_mesurements.push_back(Eigen::Vector2d(-7,-10));
    this->_mesurements.push_back(Eigen::Vector2d(-2,-9));

}

void unit_test::test_extract_pts(void){

    sample_points();//setting sample points

    double correctpts=3;

    std::vector<bloom_filter::gaussian_component> result;

    this->extract_relvant_points(result);

    cout<<"results = "<< result.size()<<endl<<result;

    if(result.size()==correctpts){
        cout<<"bloom_filter::extract_relvant_points test == Passed"<<endl;
    }
    else{cout<<"bloom_filter::extract_relvant_points test == failed"<<endl;}

    cout<<"remaining = "<< _belief.size()<<endl<<_belief;

}

void unit_test::test_measurment_update(void){

    sample_points();

    std::vector<bloom_filter::gaussian_component> extracted_points;
    

    this->extract_relvant_points(extracted_points);

    cout<<"Printing extracted points.............."<<endl;
    this->print_list(extracted_points);

     cout<<"Printing measurements.............."<<endl;
     print_list(_mesurements);

    this->measurement_update(extracted_points);
    cout<<"Printing measurement updated points.............."<<endl;
    this->print_list(extracted_points);
    cout<<extracted_points<<endl;



}

void unit_test::test_multiple_measurment_update(void){
    sample_points();

    std::vector<bloom_filter::gaussian_component> extracted_points;
    

    this->extract_relvant_points(extracted_points);

        for(int i=0;i<1000;i++){

            cout<<".........run number = "<<i<<endl;

            cout<<"Printing extracted points.............."<<endl;
            this->print_list(extracted_points);

            cout<<"Printing measurements.............."<<endl;
            print_list(_mesurements);

            this->measurement_update(extracted_points);
            cout<<"Printing measurement updated points.............."<<endl;
            this->print_list(extracted_points);
            cout<<extracted_points<<endl;


        }
}

void unit_test::test_callback_usv(ros::NodeHandle _n){

    sample_points();

    ros::Publisher tempPub=_n.advertise<sensor_msgs::Temperature>(this->_USV_topicname,10);

    sensor_msgs::Temperature t; 
    t.temperature=6;

    ros::Rate loop_rate(10);

    int counter=0;

    while(ros::ok()){

        loop_rate.sleep();

        tempPub.publish(t);

        ros::spinOnce();

        if(counter>1){break;}

        counter++;

    }
}

void unit_test::test_writefile(void){

    //sample_points();
    _filepath="/home/thivanka/catkin_ws/src/uri_soft_wip/cyanobloom/cyanob_phd_filter/src/coordinates.txt";
    cout<<"file path"<<_filepath<<endl;
    write2file(&_belief);
   // cout<<"file path"<<_filepath<<endl;
    //bloom_filter::readfile::_filepath =std::to_string(std::filesystem::current_path()) ;

}

void unit_test::test_read_record2list(void){
    //_filepath="/home/thivanka/catkin_ws/src/uri_soft_wip/cyanobloom/cyanob_phd_filter/src/test_coord.txt";
     cout<<"file path "<<_filepath<<endl;
     this->print_list(_belief_bar_list);

    std::vector<gaussian_component> test;
     record2list(&test);
    double t1;
    //get_data (test);
    
}




int main (int argc, char** argv){

    ros::init(argc, argv,"testing_phd_filter");

    ros::NodeHandle n;

    ros::Rate(double (1));

    std::unique_ptr<unit_test> test(new unit_test(n));

    cout << "Size of int : " << sizeof(int) << endl;

    //test->test_writefile();

    //test->test_read_record2list();

    //test->test_callback_usv(n);

    //test->test_extract_pts();

    //test->test_measurment_update();

    //test->test_multiple_measurment_update();

    //bloom_filter::phdfilter test=n;

    //std::unique_ptr<bloom_filter::phdfilter> uptr(new bloom_filter::phdfilter(n));

    n.~NodeHandle();

    ros::shutdown();

    return 0;
}