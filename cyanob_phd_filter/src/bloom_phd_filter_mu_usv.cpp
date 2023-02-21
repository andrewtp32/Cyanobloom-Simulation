#include "bloom_phd_filter.hpp"


void bloom_filter::phdfilter::callback_usv_meas(const sensor_msgs::Temperature::ConstPtr& msg){
    
    vector<int> list_pts_insidecircle;//(sz);
    list_pts_insidecircle.reserve(_belief.size());
    double radius_for_bloomconst = 10;
    //radius_for_bloomconst = 10;
    geometry_msgs::Point32 center; center.x=0; center.y=0;
/*must provide radius to consider, positon in the msg*/
    double sumW=0;

    cout<<"bloom concentration = "<<msg->temperature<<endl;
    
    for (unsigned int i =0;i < this->_belief.size();i++){

        double r = hypot((center.x-_belief[i].mean.x()),(center.y-_belief[i].mean.y()));
        //cout<<"bel weight "<<i<<" = "<<_belief[i]._w<<endl;

        if (r<radius_for_bloomconst){

            list_pts_insidecircle.push_back(i);
            sumW+=_belief[i]._w;

            // std::cout << "size: " << (int) list_pts_insidecircle.size();
            // std::cout << ", capacity: " << (int) list_pts_insidecircle.capacity() << '\n';
        }
    }

    if (sumW>0){

        double ratio = msg->temperature/sumW;

        cout<<"total weights = "<<sumW<<", ratio = "<<ratio<<endl;

        for (unsigned int j =0;j <list_pts_insidecircle.size();j++){

            _belief[list_pts_insidecircle[j]]._w*=ratio;

            //double r = hypot((center.x-_belief[i].mean.x()),(center.y-_belief[i].mean.y()));

            // if (r<radius_for_bloomconst){
            //     _belief[i]._w=_belief[i]._w*ratio;
            // }
        }
            

    }


}

