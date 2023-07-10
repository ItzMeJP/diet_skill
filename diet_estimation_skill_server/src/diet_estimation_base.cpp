#include <diet_estimation_skill_server/diet_estimation_base.h>


namespace diet_estimation_skill {

    /// <summary>
    /// Contructor
    /// </summary>
    DietEstimationBase::DietEstimationBase(){};

    /// <summary>
    /// Destructor
    /// </summary>
    DietEstimationBase::~DietEstimationBase(){};


    /// <summary>
    /// Setup static configuration for heuristic base
    /// </summary>
    /// <param name="_node_handle"> public node handle.</param>
    /// <param name="_private_node_handle"> private node handle.</param>
    /// <param name="_configuration_namespace"> ros param configuration namespace.</param>
    void DietEstimationBase::setupBaseConfigurationFromParameterServer (ros::NodeHandlePtr &_node_handle,
                                                                              ros::NodeHandlePtr &_private_node_handle,
                                                                              std::string _configuration_namespace) {

        node_handle_ = _node_handle;
        private_node_handle_ = _private_node_handle;
        _private_node_handle->param<double>(_configuration_namespace + "weight", weight_, 1.0);
    }

    /// <summary>
    /// Get the result of current heuristic processing
    /// </summary>
    DietEstimationBase::CandidateList DietEstimationBase::getResultList(){
        return candidate_list_;
    }

    /// <summary>
    /// Set the initial result of current heuristic processing
    /// </summary>
    /// <param name="_configuration_namespace"> the input list.</param>
    void DietEstimationBase::setInitialList(CandidateList _input_list){
        candidate_list_ = _input_list;
    }


}
