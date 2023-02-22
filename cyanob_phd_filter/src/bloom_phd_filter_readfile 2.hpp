#ifndef BLOOM_PHD_FILTER_READFILE_HPP
#define BLOOM_PHD_FILTER_READFILE_HPP


#include <iostream>
#include <chrono>
#include <ctime>
#include <fstream>
#include <istream>
#include <typeinfo>  
#include "bloom_phd_filter_gaussianMixture.hpp"
#include <dirent.h>



using namespace std;

namespace bloom_filter{

    class readfile {
        
    protected:

        std::fstream _file;

        std::string _filepath; //file path name

        sensor_msgs::PointCloud* _pointcld_ptr; //store to point cloud msg

        std::vector<gaussian_component> _belief_bar_list;

        std::vector<gaussian_component>* _ptr2list; //pointer to outside list

        inline friend std::istream& operator>> (std::istream& in, readfile& p);

        void record2list(std::vector<gaussian_component>* ptr);// read data and store 

        virtual void validate_file ();//read the file and check 

        void record2PC();// store to a point cloud

        template<typename T>
        void get_data(T* list_ptr);
        //template<> void get_data(sensor_msgs::PointCloud*);

        enum list_datatype {gc,pc}; //gc gaussian, pc pointcloud msg



     
    public:

        readfile(){}

        /*Initiate using a list and file path*/
        //used in the cyanob filter
        readfile(std::vector<gaussian_component>* list,std::string path);

        /*initiate using direct file path*/
        readfile(std::string filename);

        /*read file path from command line*/
        readfile(sensor_msgs::PointCloud* ptcloud);

        /*file path given in the launch file*/
        readfile(sensor_msgs::PointCloud* ptcloud,std::string filename);

        /*For parallel filter to save constants*/
        void save_PPHDFilterConst (std::string PathnFilename,int size,string arrayofsentences[]);
   

        ~readfile();

        void write2file(const std::vector<gaussian_component>* list,bool isrecordMeasurement=false);// write back to a text file
    
        string get_fpath(); //return the file path

        virtual void record2file();//write data to a file
    };



    //Cin overload
    std::istream& operator>> (std::istream& in, readfile& p){

        std::cout<<"enter file name : ";

        in>>p._filepath;

        return in;

    }
}

    

#endif