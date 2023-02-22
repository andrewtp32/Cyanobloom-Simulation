#include "parallel_filter.hpp"
using namespace std;
namespace bloom_filter{

    parallel_filter::parallel_filter(ros::NodeHandle n):_nh(n){

        _measurementupdateactivated=false;
        _pruneWeight=0.1; //Prune weight limit to time update function
        _AllmeasSize=0; //std::vector size for _allMeasurement that save measurements.
        
        set_Pd(0.7);// setting the probability of detection. default value is =0.6
        set_prune_weight();//default is 0.001


        //this->_n=n;
        string _sysdiagTopicname; 
        string _GroundTruthFolder;
        _nh.param<string>("diaganostic_topicName",_sysdiagTopicname,"/system_Diagnostics");//system diagnostic topic name
        _nh.param<string>("parallelfilter/file_path",_fname,"initial_ground_truth.txt");
        _nh.param<string>("parallelfilter/folder_path",_folderpath,"~/catkin_ws/src/uri_soft_wip/cyanobloom/cyanob_phd_filter/results/test/");
        _nh.param<double>("parallelfilter/noise",_noise,0.0);
        _nh.param<int>("parallelfilter/wind",_f,0);
        _nh.param<int>("parallelfilter/wind_angle",_ftheta,0);//starting north clockwise
        _nh.param<int>("parallelfilter/temp",_t,0);
        _nh.param<string>("meas_topic_name",_submtn,"/bloom_locations_topic");//camera results
        _nh.param<string>("parallelfilter/results_topicName",_pubtn,"filter_results");//topic name to publish results
        _nh.param<string>("bloomLoc_fpath",_GroundTruthFolder,"Error");//Ground Truth - which blooms were used
        //_n.param<string>("drone_pose",_sub_dpose,"/drone_0/gt_pose");//drone pose
        //_n.param<string>("usv_measurement",_USV_topicname,"/bloom_concentration_value_at_USV_location");//measurements from USV
        //cout<<"filter noise topic name- "<<_pubtn<<endl;

       /*This part will save the filter parameters into a textfile*/ 
            string filterparams[5]={"Noise ="+to_string(_noise),
                                    "Wind F ="+to_string(_f),
                                    "Wind angle ="+to_string(_ftheta),
                                    "Tempreture ="+to_string(_t),
                                    "GroundTFolder ="+_GroundTruthFolder
                                    };
            readfile recordparam;
            recordparam.save_PPHDFilterConst(_folderpath+"___Filter_Parameters___",5,filterparams);

        _isDroneAltitudesound=true;
        _isMUrequested=false;

        /*----Python interpriter initiating---------*/
        _pythonFileName="time_update_parallel";//python file name that contain time update class
        _pythonclassName="time_update_parallel";//python class name
        Py_Initialize() ;
        PyErr_Print();
        cout<<"PFCPP->tu initalized"<<endl;
        PyRun_SimpleString(
            "import sys \n"
            "import os \n"
            "sys.path.append(os.path.expanduser('~/catkin_ws/src/uri_soft_wip/cyanobloom/cyanob_phd_filter/scripts') )\n"
        );
        cout<<"PFCPP->tu path appended"<<endl;
        PyErr_Print();
        pmodule=PyImport_Import(PyString_FromString(_pythonFileName.c_str())); //importing timeupdate class
        if(pmodule==NULL){
            cout<<"PFCPP->pyimport failed!!!"<<endl;
            PyErr_Print();
            exit(-1);
        }
        cout<<"PFCPP->tu class successfully imported"<<endl;
        /*----Python interpriter initiating end---------*/



        _belief.clear();//stores previous estimates which reads via file


        //std::unique_ptr<readfile> _initial_points_file2(new readfile(&_belief,_fpath));
        //_initial_points_file= std::shared_ptr<readfile> (new readfile(&_belief,_fpath));

        cout<<"PFCPP->Measurement update node ready.. "<<_belief.size()<<endl;
        //print_list(_belief);
        //cout<<_belief;

        bloom_filter::phdfilter* phdptr;


        _sub=_nh.subscribe(_submtn, 100,&bloom_filter::parallel_filter::callback_camera_meas2,this);


        // _sub_dronepose=_n.subscribe(_sub_dpose, 10, &bloom_filter::phdfilter::callback_dronepose, this);

        _sub_SDiagnostics=_nh.subscribe(_sysdiagTopicname,2,&bloom_filter::parallel_filter::callback_sys_diag,this);
        
        _pub=_nh.advertise<sensor_msgs::PointCloud>(_pubtn,10); //publisher for results.

        // _pub_diagnos=_n.advertise<cyanob_phd_filter::diagnostics>("system_Diagnostics",5,true);

        //cout<<"PFCPP->Measurement update end.. "<<_belief.size()<<endl;
    }

    void bloom_filter::parallel_filter::callback_camera_meas2(const sensor_msgs::PointCloud::ConstPtr& msg){
        //cout<<"PFCPP->inside callback camera.. "<<_belief.size()<<endl;
        if(_isMUrequested==true){

            std::vector<gaussian_component> * _meas_ptr, pointsInsideImg, * msupdatelistptr;

            _meas_ptr=&_mesurements;

            _meas_ptr->clear();

            _msg_filter_results.points.clear();

            int count=0,count_rand=0;
            std::vector<int> listwithrandomNumbers;
            if(msg->points.size()>4){

                int pointsLength= msg->points.size();
                int percentage=(pointsLength-4)*(_Pd);//because i am removing 1-pd percentage by erasing only this much

                //cout<<"[measurements size] - "<<pointsLength<<", [percentage] - "<<percentage<<", [Pd]- "<<1-_Pd <<endl;

                for (int i=4; i<pointsLength; ++i) listwithrandomNumbers.push_back(i); // 1 2 3 4 5 6 7 8 9
                // using built-in random generator:
                std::random_shuffle ( listwithrandomNumbers.begin(), listwithrandomNumbers.end() );
                listwithrandomNumbers.erase (listwithrandomNumbers.begin(),listwithrandomNumbers.begin()+percentage);
                listwithrandomNumbers.shrink_to_fit();
                std::sort(listwithrandomNumbers.begin(), listwithrandomNumbers.end());
                // cout<<"[random number] - ";
                // for (int i=0;i<listwithrandomNumbers.size();i++){
                    
                //     cout<<","<<listwithrandomNumbers[i];
                // }
                // cout<<" end "<<endl;


            }



            for (const geometry_msgs::Point32& _iter:msg->points){

                if (count<4){// recording four corners of the image
                    _imgcornerpts[count]=_iter;
                    //count++;//counter will advance until 4.
                    //cout<<"printing corner , "<<count<<", "<<_iter<<endl;
                }
                else{
                    
                    
                    if(count_rand<listwithrandomNumbers.size() && listwithrandomNumbers[count_rand]==count ){

                        //cout<<"[ignored]random index "<<listwithrandomNumbers[count_rand]<<", counter -"<<count<<endl;
                        count_rand++;
                        count++;
                        continue;
                        //cout<<"continue doesnt work" <<endl;
                    }
                    //cout<<"random index "<<listwithrandomNumbers[count_rand]<<", counter -"<<count<<endl;
                    _meas_ptr->push_back(gaussian_component{Eigen::Vector2d(_iter.x,_iter.y)});
                    _Allmesurements.push_back(gaussian_component{Eigen::Vector2d(_iter.x,_iter.y)}); //save measurements to the container

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
            //cout<<"camera call back end to publish"<<endl;
            this->publish();
            
        }
    }

    //virtual function overlaod from filte_measurement_update
    void parallel_filter::callback_sys_diag(const cyanob_phd_filter::diagnostics::ConstPtr& msg){
        cout<<"PFCPP->MU_callback_sys_diag = printing from inside the parallel filter"<<endl;

        
        cyanob_phd_filter::diagnostics newmsg;
        newmsg=*msg; //taking a copy of the latest msg

        if (msg->time_update== "activate"&&_measurementupdateactivated==false ){
            
            _TuFilepath=time_update(); //call time update functiona and retrive the path

            
            //cout<<"measurement update msg received. status = "<<msg->measurement_update<<endl;
            cout<<"PFCPP->begin measurement update"<<endl;
            _Allmesurements.clear(); //clear the measurements recorder
            if(_AllmeasSize>0) _Allmesurements.reserve(_AllmeasSize); //_AllmeasSize is the size of last container
            _initial_points_file.reset();

            


            try{
                _measurementupdateactivated=true; //provide MU to run
                _initial_points_file= std::shared_ptr<readfile> (new readfile(&_belief,_TuFilepath));
                _isMUrequested=true; //giving permission to do MU
                //newmsg.measurement_update="running";
                // newmsg.simulation="activate";
                // _pub_diagnos.publish(newmsg);

            }
            catch (...){
                cout<<"PFCPP->error catched in call back system diagnose inside paralle_filter.cpp"<<endl;
                newmsg.measurement_update="PFCPP->file error in measurement update";
                //_pub_diagnos.publish(newmsg);

            }

        }

        else if(msg->measurement_update== "stop" && msg->simulation== "complete"){//this should come from the simulation
            
            cout<<"PFCPP->measurement update status = "<<msg->measurement_update<<endl;
            cout<<"PFCPP->received stop msg"<<endl;
            cout<<"PFCPP->Saving MU points to a file"<<endl;
            _isMUrequested=false; //stopping mu
            _initial_points_file->write2file(&_belief); //save results.
            _initial_points_file->write2file(&_Allmesurements,true);//save raw measurements provided by camera
            _AllmeasSize=_Allmesurements.size();
            cout<<"PFCPP-> measurments vector size = "<<_AllmeasSize<<endl;
            ///TODO change in write2file -> erase file  before write. Do not append it will cause error in time update when reading the file
            string fpth (_initial_points_file->get_fpath());
            std::size_t pos = fpth.find_last_of("/");
            fpth.erase(0,pos+1); //erasing system path and have only the name of the file inside the result folder.
            _fname=fpth;//TU will use this file name in next time
            cout<<"PFCPP->path ->"<<fpth<<endl;

            // newmsg.filename=fpth;
            // newmsg.measurement_update="ready";
            // newmsg.bloom_pub="activate";
            // _pub_diagnos.publish(newmsg);
             _measurementupdateactivated=false;
             

        }

    }

    void parallel_filter::print_list(std::vector<gaussian_component>& l){
        cout<<"PFCPP->printing inside the parallel filter"<<endl;

    }

    string parallel_filter::time_update(void){
        // execute python script in here.
        //PyObject *pmodule,*myclass;
        cout<<"PFCPP->tu inside"<<endl;
        // Py_Initialize() ;
        // cout<<"PFCPP->tu initalized"<<endl;
        // PyRun_SimpleString(
        //     "import sys \n"
        //     "import os \n"
        //     "sys.path.append(os.path.expanduser('~/catkin_ws/src/uri_soft_wip/cyanobloom/cyanob_phd_filter/scripts') )\n"
        //     //"from time_update_parallel import * \n"
        //     //"tup=time_update_parallel('/home/thiw1ka/catkin_ws/src/uri_soft_wip/cyanobloom/cyanob_phd_filter/results/',0.1,'initial_ground_truth.txt',10,1,90,30)"
        //     );
        // cout<<"PFCPP->tu path appended"<<endl;
        // //PyErr_Print();
        // PyObject* pmodule=PyImport_Import(PyString_FromString(_pythonFileName.c_str()));
        // if(pmodule==NULL){
        //     cout<<"PFCPP->pyimport failed!!!"<<endl;
        //     PyErr_Print();
        //     exit(-1);
        // }

        // cout<<"PFCPP->tu class imported"<<endl;
        PyObject* myclass=PyObject_GetAttrString(pmodule,_pythonclassName.c_str());
        if(myclass==NULL){
            cout<<"PFCPP->pyimport failed!!!"<<endl;
            PyErr_Print();
            exit(-1);
        }
        //cout<<"PFCPP->tu class function"<<endl;
        //pArgs=(7,PyString_FromString(cstrfoldername),PyFloat_FromDouble(0.1),PyString_FromString((char*)cstrFilename),PyFloat_FromDouble(10),PyFloat_FromDouble(1),PyFloat_FromDouble(90),PyFloat_FromDouble(30));
        PyObject* args = PyTuple_Pack(7,PyString_FromString(_folderpath.c_str()),PyFloat_FromDouble(_pruneWeight),PyString_FromString(_fname.c_str()),PyFloat_FromDouble(_noise),PyFloat_FromDouble(_f),PyFloat_FromDouble(_ftheta),PyFloat_FromDouble(_t));
        PyErr_Print();
        //cout<<"PFCPP->tuple"<<endl;
        PyObject* results=PyObject_CallObject(myclass, args);
        PyErr_Print();
        //cout<<"PFCPP->objeect called"<<endl;
        PyObject* filepath=PyObject_CallMethod(results,"get_path",NULL);
        PyErr_Print();
        //cout<<"PFCPP->tu class results requested"<<endl;
        const char* r = PyString_AsString(filepath);
        std::string str(r);
        cout<<"PFCPP->filename returned from TU="<<str<<endl;


        //cleaning up python objects in memory
        Py_DECREF(filepath);
        PyErr_Print();
        Py_DECREF(results);
        Py_DECREF(args);
        Py_DECREF(myclass);
        PyErr_Print();
        // Py_DECREF(pmodule);


	    // Py_Finalize();//this will call destructor for TU class
        // cout<<"PFCPP-> python interpretor closed"<<endl;
        return str;
    }

    parallel_filter::~parallel_filter(void){
        cout<<"parallel filter destructor called"<<endl;
        Py_DECREF(pmodule);
        PyErr_Print();
        Py_Finalize();//this will call destructor for TU class
        cout<<"PFCPP-> python interpretor closed"<<endl;

    }

}





int main (int argc, char *argv[]){

    ros::init(argc, argv, "parallel_filter_node");

    ros::NodeHandle nh;

    ros::AsyncSpinner spinner(0);


    shared_ptr<bloom_filter::parallel_filter> pf (new bloom_filter::parallel_filter(nh));

    spinner.start();

    // string folPath = "/home/thiw1ka/catkin_ws/src/uri_soft_wip/cyanobloom/cyanob_phd_filter/results/test/";
    // string filename = "initial_ground_truth.txt";



    // shared_ptr<bloom_filter::phdfilter> bf=pf;

    // std::vector<bloom_filter::gaussian_component> test;

    // test.push_back(bloom_filter::gaussian_component(Eigen::Vector2d(-100,-10),1.0));

    // pf->print_list(test);

    // bf->print_list(test);//print used in parallel filter because of the virtual function

    

    printf("\n PFCPP->Paralle filter started...\n");

    ros::waitForShutdown();

    return 0;
}