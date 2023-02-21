#include "bloom_phd_filter.hpp"

double bloom_filter::phdfilter::area_triang (geometry_msgs::Point32& corner1,geometry_msgs::Point32& corner2, double x3, double y3 ){
    //area of an triangle using 3 coordinate points
    return 0.5*(abs(corner1.x*(corner2.y-y3)+corner2.x*(y3-corner1.y)+x3*(corner1.y-corner2.y)));

}

double bloom_filter::phdfilter::get_total_area(gaussian_component& point){
//calculating the area of four triangles
    Eigen::Vector2d m(point.getMean());
    double tot=area_triang(_imgcornerpts[0],_imgcornerpts[1],m(0),m(1));
    tot+=area_triang(_imgcornerpts[0],_imgcornerpts[2],m(0),m(1));
    tot+=area_triang(_imgcornerpts[1],_imgcornerpts[3],m(0),m(1));
    tot+=area_triang(_imgcornerpts[2],_imgcornerpts[3],m(0),m(1));
    return trunc(tot);
}

/*this function will extract points and add them to specified list*/
/*uses internal _imgcornerpts list to define four corners*/
void bloom_filter::phdfilter::extract_relvant_points (std::vector<gaussian_component>& points_extracted){
    //0 top left, 1 top right, 2 bottom left, 3 bottom right,, 
    //0&1 are longer side,, 0&2 width

    double length = hypot((_imgcornerpts[1].x-_imgcornerpts[0].x) , (_imgcornerpts[1].y-_imgcornerpts[0].y));
    double width = hypot((_imgcornerpts[2].x-_imgcornerpts[0].x) , (_imgcornerpts[2].y-_imgcornerpts[0].y));
    double area_sq = trunc(length * width);

    //cout<<"length = "<<length<<" ,width = "<<width<<" ,area = "<<area_sq<<endl;

    vector<unsigned int> list2delete;//recoding indexes of extracted points

    list2delete.clear();

    for (unsigned int i=0;i<_belief.size();i++){

        //cout<<"total area belief of "<<i<<" , area = " <<get_total_area(_belief[i])<<",logic = "<<((area_sq)==get_total_area(_belief[i]))<<endl;

        /*if a point is inside then total area is equal to the square*/
        if(area_sq==get_total_area(_belief[i])){

            points_extracted.push_back(_belief[i]);

            list2delete.push_back(i);
            //cout<<"pushedback = "<<i<<endl;
            //_belief.erase(priorpt);
        }
    }
    //cout<<"number of extracted points = "<<points_extracted.size()<<endl;
    // //sorting the indexes so erasing can start from the bottom
    // std::sort(list2delete.begin(),list2delete.end());
    // cout<<"sorted = ";
    // for(int it:list2delete){cout<<it;}cout<<endl;

    //extracted points will be erased from original list.
    //start erasing from bottom of the list.
    for (std::vector<unsigned int>::reverse_iterator it = list2delete.rbegin(); it != list2delete.rend(); ++it){
       //cout<<"deleting = "<<*it<<endl;
        _belief.erase(_belief.begin() + * it);

    }
}

void bloom_filter::phdfilter::set_Pd (double p){
    _Pd=p;
}



void bloom_filter::phdfilter::set_prune_weight (double pw){
    _pruneW=pw;
}

void bloom_filter::phdfilter::callback_dronepose(const geometry_msgs::Pose::ConstPtr& msg){
    double height =msg->position.z;

    if(height>14){
        _isDroneAltitudesound=true;
    }
    else{
        _isDroneAltitudesound=false;
        //std::cout<<"..................drone altitude is low!!!!! -> "<<height<<endl;
       
    }
}

void bloom_filter::phdfilter::callback_camera_meas(const sensor_msgs::PointCloud::ConstPtr& msg){
    cout<<"inside callback camera.. "<<_belief.size()<<endl;
    if(_isDroneAltitudesound==true&&_isMUrequested==true){

        std::vector<gaussian_component> * _meas_ptr, pointsInsideImg, * msupdatelistptr;

        _meas_ptr=&_mesurements;

        _meas_ptr->clear();

        _msg_filter_results.points.clear();

        int count=0;

        for (const geometry_msgs::Point32& _iter:msg->points){

            if (count<4){// recording four corners of the image
                _imgcornerpts[count]=_iter;
                //count++;//counter will advance until 4.
                //cout<<"printing corner , "<<count<<", "<<_iter<<endl;
            }
            else{

                _meas_ptr->push_back(gaussian_component{Eigen::Vector2d(_iter.x,_iter.y)});

            }
            _msg_filter_results.points.push_back(_iter); //pushing all points into filter results msg
            count++;//counter will advance until 4.
        }

        //adding details where measurements ends
        this->_msg_filter_results.header.frame_id="meas_ends="+to_string(count);

        //cout<<_msg_filter_results.header.frame_id<<endl;
        
        /*adding the prior estimates*/
        for(gaussian_component _iter:_belief){
            _msg_filter_results.points.push_back(_iter.get_geomtryPt());
            count++;
        }
        //adding details(index) where measurement points ends
        this->_msg_filter_results.header.frame_id+=",prior="+to_string(count);

        pointsInsideImg.clear();
        extract_relvant_points(pointsInsideImg);//extracting points that are inside the current image
        measurement_update(pointsInsideImg);//do meas update to extracted points and save it 
        
        double c1=count;
        /*adding the posterior estimates to filter result msg*/
        for(gaussian_component _iter:pointsInsideImg){
            _msg_filter_results.points.push_back(_iter.get_geomtryPt());
            count++;
        }
        //adding details where measurements ends
        this->_msg_filter_results.header.frame_id+=",post="+to_string(count);
       // cout<<"number of filter estimates = "<<(count-c1)<<", belief size ="<<_belief.size()<<endl;

        /*publish results to a topic*/
        this->publish();
        
    }
    //else{cout<<"drone altitude is not correct..........................."<<endl;}


}

void bloom_filter::phdfilter::callback_sys_diag(const cyanob_phd_filter::diagnostics::ConstPtr& msg){
    cyanob_phd_filter::diagnostics newmsg;
    newmsg=*msg; //taking a copy of the latest msg

    if (msg->measurement_update== "activate" ){
	    cout<<"measurement update msg received. status = "<<msg->measurement_update<<endl;
        cout<<"begin measurement update"<<endl;
	    _initial_points_file.reset();


        try{
            _initial_points_file= std::shared_ptr<readfile> (new readfile(&_belief,msg->filename));
            _isMUrequested=true; //giving permission to do MU
            newmsg.measurement_update="running";
            newmsg.simulation="activate";
            _pub_diagnos.publish(newmsg);

        }
        catch (...){
            cout<<"error catched"<<endl;
            newmsg.measurement_update="file error";
            _pub_diagnos.publish(newmsg);

        }

    }
    else if (msg->measurement_update== "running"){
        cout<<"measurement update waiting for simulation to begin.."<<endl;

    }
    else if (msg->measurement_update== "request_status"){
        cout<<"measurement update status requested"<<endl;
        newmsg.measurement_update="ready";
        _pub_diagnos.publish(newmsg);
    }
    else if(msg->measurement_update== "stop" && msg->simulation== "complete"){//this should come from the simulation
		cout<<"measurement update status = "<<msg->measurement_update<<endl;
		cout<<"received stop msg"<<endl;
		cout<<"Saving MU points to a file"<<endl;
		_isMUrequested=false; //stopping mu
		_initial_points_file->write2file(&_belief); //save results.
        ///TODO change in write2file -> erase file  before write. Do not append it will cause error in time update when reading the file
		string fpth (_initial_points_file->get_fpath());
        std::size_t pos = fpth.find_last_of("/");
        fpth.erase(0,pos+1); //erasing system path and have only the name of the file inside the result folder.
        cout<<"path ->"<<fpth<<endl;
        newmsg.filename=fpth;
		newmsg.measurement_update="ready";
		newmsg.bloom_pub="activate";
		_pub_diagnos.publish(newmsg);

	}


}

void bloom_filter::phdfilter::publish(void){
    //cout<<"Here to publish inside bloom phd cpp publish"<<endl;
   _pub.publish(_msg_filter_results);
   //cout<<"Here to publish inside bloom phd cpp publish_2"<<endl;
}

void bloom_filter::phdfilter::print_list(std::vector<gaussian_component>& l){
    cout<<"list start....."<<endl;
    for (gaussian_component _it:l){
        //Eigen::Vector2d m=_it.getMean();
        cout<<"x,y ="<<_it.mean.x()<<","<<_it.mean.y()<<" , weight "<<_it.getweight()<<endl;
    }
    cout<<"list end....."<<endl;
}

 bloom_filter::phdfilter::phdfilter(ros::NodeHandle n):_n(n){

     cout<<"phd filter constructor"<<endl;

     //file that contain previous estimates.
     _n.param<string>("readfile/file_path",_fpath,"/home/thiw1ka/catkin_ws/src/uri_soft_wip/cyanobloom/cyanob_phd_filter/results/coordinates.txt");
     _n.param<string>("meas_topic_name",_submtn,"/bloom_locations_topic");//topic to subscribe
     _n.param<string>("Publish_results_topic",_pubtn,"filter_results");//topic name to publish results
     _n.param<string>("drone_pose",_sub_dpose,"/drone_0/gt_pose");//drone pose
     _n.param<string>("usv_measurement",_USV_topicname,"/bloom_concentration_value_at_USV_location");//measurements from USV
     
     _isDroneAltitudesound=false;
	 _isMUrequested=false;


     _belief.clear();//stores previous estimates which reads via file

     set_Pd();// setting the probability of detection. default value is =0.99

     set_prune_weight();//default is 0.5

    //std::unique_ptr<readfile> _initial_points_file2(new readfile(&_belief,_fpath));
     //_initial_points_file= std::shared_ptr<readfile> (new readfile(&_belief,_fpath));

     cout<<"Measurement update node ready.. "<<_belief.size()<<endl;
    //print_list(_belief);
    //cout<<_belief;

     _sub=_n.subscribe(_submtn, 100, &bloom_filter::phdfilter::callback_camera_meas, this);

     _sub_dronepose=_n.subscribe(_sub_dpose, 10, &bloom_filter::phdfilter::callback_dronepose, this);

     _sub_USV_mu=_n.subscribe(_USV_topicname, 10, &bloom_filter::phdfilter::callback_usv_meas, this);

     _sub_diagnos=_n.subscribe("system_Diagnostics",2,&bloom_filter::phdfilter::callback_sys_diag, this);
     
     _pub=_n.advertise<sensor_msgs::PointCloud>(_pubtn,10);

     _pub_diagnos=_n.advertise<cyanob_phd_filter::diagnostics>("system_Diagnostics",5,true);

     //_belief=_initial_points_file::get_points();

     //_timer1= _n.createTimer(ros::Duration(1.0),&bloom_filter::phdfilter::measurement_update,this);
     //Timer createTimer(Rate r, Handler h, Obj o, bool oneshot = false, bool autostart = true) const
    //timer1 = _n.createTimer(ros::Duration(0.3),&PHDfilter::run_filter,this)
}

bloom_filter::phdfilter::phdfilter(){

}


bloom_filter::phdfilter::~phdfilter(){
    std::cout<<"phd filter destructor"<<std::endl;
    //_initial_points_file->write2file(&_belief); //save results.
    _mesurements.~vector();
    _belief.~vector();
    _msg_filter_results.points.clear();
    //if(_initial_points_file) delete _initial_points_file;
    _n.shutdown();
    _n.~NodeHandle();
    
}
