#include <diet_estimation_skill_server/diet_estimation_base.h>


namespace diet_estimation_skill {

    DietEstimationBase::DietEstimationBase(){};

    DietEstimationBase::~DietEstimationBase(){};



    void DietEstimationBase::setupBaseConfigurationFromParameterServer (ros::NodeHandlePtr &_node_handle,
                                                                              ros::NodeHandlePtr &_private_node_handle,
                                                                              std::string _configuration_namespace) {

        node_handle_ = _node_handle;
        private_node_handle_ = _private_node_handle;
        _private_node_handle->param<double>(_configuration_namespace + "weight", weight_, 1.0);
    }


    DietEstimationBase::CandidateList DietEstimationBase::getResultList(){
        return candidate_list_;
    }

    void DietEstimationBase::setInitialList(CandidateList _input_list){
        candidate_list_ = _input_list;
    }

    /*void DietEstimationBase::setFoodCandidates(FoodCandidates _input_candidates){
         candidates_list_ = _input_candidates;
    }
     */


}
