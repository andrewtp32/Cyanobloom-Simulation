#include "bloom_phd_filter_readfile.hpp"

namespace bloom_filter{

    void readfile::record2file(){
        cout<<"inside the readfile class"<<endl;
    }
    

    /*Constructor::Initiate using a list*/
    readfile::readfile(std::vector<gaussian_component>* list,std::string path):_filepath(path),_ptr2list(list){

        //std::cin>>*this; //obtaining the outside file name
        //readfile(path);

        validate_file();

        record2list(list);// recording to a gaussian list

    }


    /*Constructor::initiate using direct file path*/
    /*record points to the internal list*/
    readfile::readfile(std::string filename):_filepath(filename){

        validate_file();

        record2list(&_belief_bar_list);// recording to a gaussian list

    }

    /*constructor for pointcloud msg*/
    /*read file path from the command line*/
    readfile::readfile(sensor_msgs::PointCloud* ptcloud):_pointcld_ptr(ptcloud){

        std::cin>>*this; //obtaining the outside file name

        validate_file();
        
        record2PC();
        

    }

    /*file path given in the launch file*/
    /*record to a point cloud*/
    readfile::readfile(sensor_msgs::PointCloud* ptcloud,std::string filename):_pointcld_ptr(ptcloud),_filepath(filename){
    
        validate_file();
        
        record2PC();//record to a point cloud

    }


    void readfile::validate_file (){

            std::cout<<"validating the file path.."<<std::endl;

            this->_file.open(_filepath);

            //checking for error
                if(_file.fail()){
                    std::cerr << "[file opening error] "<<_filepath<<std::endl;
                    std::cerr << "file opening error. Exiting..."<<std::endl;
                    throw 2;
                    std::exit(1);
                }


           std::cout<<"File validated.."<<std::endl;

    }


     template<typename T>
     void readfile::get_data(T* list_ptr){ 

         cout<<"The data type is = "<< typeid(*list_ptr).name() <<endl;
         //cout<<"the type is point cloud = "<<( typeid(sensor_msgs::PointCloud) == typeid(list) )<<endl;
         list_datatype ldtype;

         if(typeid(*list_ptr)==typeid(vector<gaussian_component>)){
             ldtype = gc;
             list_ptr->clear();
         }
         else if (typeid(*list_ptr)==typeid(sensor_msgs::PointCloud)){
             ldtype = pc;
             //list_ptr->points.clear();
         }
         else {
             std::cerr << "Unknown data container. check {readfile::get_data function declaration}. exiting.. " << '\n';
             exit(1);
         }

        double x,y,w;
        string line;

        try{

            while(!this->_file.eof()){

                //_file>>x;
                //_file>>y;
                w=1;
                getline(_file,line);
                std::size_t notint =line.find("X");
                if (notint!=std::string::npos) continue;
                cout<< "the line = "<<line<<endl;

                std::string::size_type sz;     // alias of size_t

                try {
                    // if no double read a exception thrown 
                    x = std::stod (line,&sz);
                }
                catch (const std::invalid_argument& ia) {
                    std::cerr << "skipping line, No double value present " << ia.what() << '\n';
                    continue;
                }

                line.erase(0,sz+1);
                y=std::stod (line,&sz);
                line.erase(0,sz+1);
                try {
                    // if no double read a exception thrown 
                    w=std::stod (line,&sz);
                }
                catch (const std::invalid_argument& ia) {
                    std::cerr << "No weight value presented " << w << '\n';
                }

                if(this->_file.eof()==true) break; //if end of line flag raised it will exit the loop

                if(_file.fail()){_file.clear();throw 01;}

                if ( (_file.rdstate() & std::ifstream::badbit ) != 0 ){throw 0;}
                save_xy:

                switch (ldtype)
                {
                    case gc:
                        list_ptr->push_back(gaussian_component(Eigen::Vector2d(x,y),w));
                        break;
                    
                    case pc:

                        //list_ptr->points.push_back(geometry_msgs::Point32(x,y,w));
                        break;
                }
                //std::cout<<"from file x = "<<x<<" , y = "<<y<<std::endl;
            }
        }catch(...){
            std::cerr<<"Error!!! invalid character present in the file. All should be type double... "<<endl;
            std::exit(1);
        }

        //std::cout<<"end of the file. Read points = "<<ptr->size()<<std::endl;

         //return true;
     }
    
     
    /*record to a gaussian components list/vector*/
    void readfile::record2list(std::vector<gaussian_component>* ptr){
        cout<<"recording to a list inside readfile class"<<endl;
       // get_data(ptr);
        // sensor_msgs::PointCloud test;
        // get_data(&test);
        ptr->clear();

        double x,y,w;
        string line;

        try{

            while(!this->_file.eof()){

                //_file>>x;
                //_file>>y;
                w=1;
                getline(_file,line);
                std::size_t notint =line.find("X");
                if (notint!=std::string::npos) continue;
                //cout<< "the line = "<<line<<endl;

                std::string::size_type sz;     // alias of size_t

                try {
                    // if no double read a exception thrown 
                    x = std::stod (line,&sz); //read number untill a space is found.
                }
                catch (const std::invalid_argument& ia) {
                    std::cerr << "skipping line, No double value present " << ia.what() << '\n';
                    continue;
                }

                line.erase(0,sz+1);//erasing till x value
                y=std::stod (line,&sz);
                line.erase(0,sz+1);
                try {
                    // if no double read a exception thrown 
                    w=std::stod (line,&sz);
                }
                catch (const std::invalid_argument& ia) {
                    //std::cerr << "No weight value presented " << w << '\n';
                }

                if(this->_file.eof()==true) break; //if end of line flag raised it will exit the loop

                if(_file.fail()){_file.clear();throw 01;}

                if ( (_file.rdstate() & std::ifstream::badbit ) != 0 ){throw 0;}
                save_xy:
                ptr->push_back(gaussian_component(Eigen::Vector2d(x,y),w));
                //std::cout<<"from file x = "<<x<<" , y = "<<y<<std::endl;
            }
        }catch(...){
            std::cerr<<"Error!!! invalid character present in the file. All should be type double... "<<endl;
            std::exit(1);
        }

        std::cout<<"end of the file. Read points = "<<ptr->size()<<std::endl;

    }

    /*recording to a point cloud*/
    void readfile::record2PC(){
        
        //get_data(_pointcld_ptr);
        _pointcld_ptr->points.clear();

        geometry_msgs::Point32 point;

        point.z=1;

        string line;

        try{
            while(!this->_file.eof()){
        
                //_file>>point.x;
                //_file>>point.y;
                point.z=1;
                getline(_file,line);
                std::size_t notint =line.find("X");
                if (notint!=std::string::npos) continue;
                //cout<< "the line = "<<line<<endl;

                std::string::size_type sz;     // alias of size_t

                try {
                    // if no double read a exception thrown 
                    point.x = std::stod (line,&sz);
                }
                catch (const std::invalid_argument& ia) {
                    std::cerr << "skipping line, No double value present " << ia.what() << '\n';
                    continue;
                }

                line.erase(0,sz+1);//erasing till x value
                point.y=std::stod (line,&sz);
                line.erase(0,sz+1);
                try {
                    // if no double read a exception thrown 
                    point.z=std::stod (line,&sz);//weight
                }
                catch (const std::invalid_argument& ia) {
                    //std::cerr << "No weight value presented. weight = " << point.z << '\n';
                }

                if(this->_file.eof()==true) break; //if end of line flag raised it will exit the loop

                if(_file.fail()){_file.clear();throw 01;}

                if ( (_file.rdstate() & std::ifstream::badbit ) != 0 ){throw 0;}
                save_xy:
                _pointcld_ptr->points.push_back(point);

                //std::cout<<"from file x = "<<point.x<<" , y = "<<point.y<<std::endl;
            }
        }
        catch(int i){
            cerr<<"Error!!!!!! "<< i <<endl<<"Invalid character present in the file. All should be type double... "<<endl;
            std::exit(1);
        }

        cout<<"successfully read "<< _pointcld_ptr->points.size() <<" points.."<<endl;


    }

    readfile::~readfile(){
        cout<<"destructor called from readfile..... "<<endl;
        _belief_bar_list.~vector();
        //if(_pointcld_ptr) delete  _pointcld_ptr;
        //if(_ptr2list) delete _ptr2list;
        _file.close();
    }

    void readfile::write2file(const std::vector<gaussian_component>* list, bool isrecordMeasurement){

         _file.close();

         string previousfilepath; //this to save original file path when saving all camera measurements

         std::size_t pos = _filepath.find_last_of("-");

        if(isrecordMeasurement==false){ _filepath.insert(pos,"_mu");}
        else{
            previousfilepath=_filepath;
             _filepath.insert(pos,"_AllCamRes");

        }

        // cout<<"file path = "<<_filepath<<endl;

         std::cout<<"Saving filter results.."<<std::endl;

         this->_file.open(_filepath,std::fstream::out|std::fstream::trunc);//all previous content deleted. use app to append.

         auto t_c = std::chrono::system_clock::to_time_t(std::chrono::system_clock::now());

         //_file<<"==================================="<<endl;


        if(_file.is_open()){
            cout<<"open successfully"<<endl;

            _file<<"X Y W"<<endl;

            for (int i=0;i<list->size();i++){

                 _file<<list->at(i).mean.x()<<" "<<list->at(i).mean.y()<<" "<< list->at(i)._w<<endl;
                 //insert weight if want
            }
            _file<<"X The results saved at "<<ctime(&t_c )<<endl;
            //_file<<"==================================="<<endl;   
   
         _file.close();

        if(isrecordMeasurement==true){
            _filepath=previousfilepath;
        }

         cout<<"[readfileCpp] measurement update results saved successfully........"<<endl;
        }
        else {cout<<"[readfileCpp] file opening error. results not saved..........."<<endl;}

    }

    string readfile::get_fpath(){ //return the file path
        return _filepath;
    }


    /*For parallel filter to save constants*/
    void readfile::save_PPHDFilterConst (std::string PathnFilename,int size,string arrayofsentences[]){

        _filepath=PathnFilename;
         std::cout<<"ReadF->Saving filter parameters.."<<std::endl;

         _file.open(_filepath,std::fstream::out|std::fstream::trunc);//all previous content deleted. use app to append.

         auto t_c = std::chrono::system_clock::to_time_t(std::chrono::system_clock::now());

         //_file<<"==================================="<<endl;


        if(_file.is_open()){
            cout<<"ReadF->open successfully"<<endl;

            _file<<"Filer parameters"<<endl;
            _file<<"==================================="<<endl; 

            for (int i=0;i<size;i++){

                 _file<<arrayofsentences[i]<<endl;
            }
            _file<<"X The results saved at "<<ctime(&t_c )<<endl;
            //_file<<"==================================="<<endl;   
   
         _file.close();

         cout<<"parameters saved successfully........"<<endl;
        }
        else {cout<<"ReadF->file opening error. param not saved..........."<<endl;}

    }






}

