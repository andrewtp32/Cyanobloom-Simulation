#include "bloom_phd_filter.hpp"


void bloom_filter::phdfilter::measurement_update(std::vector<gaussian_component>& bel_extract){//takes prior estimates relvant to current image

    if(_isMUrequested==true){ //not activated via diagnostic msg.
        //cout<<"Measurement update cpp inside.. "<<_belief.size()<<endl;
        std::vector<gaussian_component> * _meas_ptr, * _bel_ptr, _temp, _temp_final_estimate;// pointer to vector containing gaussian components

        _temp.clear();

        _temp_final_estimate.clear();

        double _weightsum=0;

        bool _isundetectedincluded=false;// for 1-pd update

        _meas_ptr=&_mesurements; //gettting the address of the measurement list

        /*reserving the size*/
        _temp_final_estimate.reserve(bel_extract.size());
        _temp.reserve(bel_extract.size());

       // std::cout<<"measurements ="<<_meas_ptr->size()<<", extracted belief ="<<bel_extract.size()<<endl;
        //print_list(_mesurements);

        for (const gaussian_component& _it_z:_mesurements){//for each measurements

            _weightsum=0;

            for (int i=0;i<bel_extract.size();i++){
            //for (gaussian_component &_it_bel:bel_extract){//for each prior estimate

                /*each component is updated with mis detection probability*/
                /*only in the first run it will update each components*/
                /*all bel_extract will be pushed into final*/
                if (_isundetectedincluded==false){

                    _temp_final_estimate.push_back(bel_extract[i]);
                    _temp.push_back(bel_extract[i]);//only weights will change in each round

                    double pdw=bel_extract[i].getweight();
                    pdw=(1-_Pd)*pdw;
                    _temp_final_estimate.back().setweight(pdw);
                    //cout<<"intial weights = "<<pdw<<endl;

                    //std::cout<<"probaility of not detecting a prior estimate weight ="<<pdw<<endl;
                }

                /*calculating weight*/
                double w=_Pd*bel_extract[i].getweight()*get_gdensity(_it_z.mean,_temp[i].mean,_temp[i].cov);
                //cout<< "calculated weight = "<<w<<endl;

                _temp[i].setweight(w);

                _weightsum+=w;
                //cout<< "weight = "<<w<< " , weight sum = "<<_weightsum<<endl;
            }

            _isundetectedincluded=true;//all the (1-pd) components been added to the final list

            for (int i=0;i<_temp.size()&& _weightsum!=0;i++){

                //_temp[i]._w=_temp[i]._w/_weightsum;
                _temp_final_estimate[i]._w+=_temp[i]._w/_weightsum;
               // cout<< "calculated weight = "<<_temp_final_estimate[i]._w<<endl;
            }

        }

        bel_extract.clear(); //clear the extracted list of points

        for (const gaussian_component& it:_temp_final_estimate){
            if(it._w>_pruneW){//pruning lower weights
            // cout<<"weight = "<<it._w<<endl;
                _belief.push_back(it);
                bel_extract.push_back(it);
            }

        }

        _temp_final_estimate.clear();
        _temp.clear();
   }
   else{cout<<"MU is not activated>>>>"<<endl;}
}