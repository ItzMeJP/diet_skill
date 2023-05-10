#include <calculator_estimation_skill_server/calculator_estimation_base.h>


namespace calculator_estimation_skill {

    CalculatorEstimationBase::CalculatorEstimationBase(){};

    CalculatorEstimationBase::~CalculatorEstimationBase(){};



    void CalculatorEstimationBase::setupBaseConfigurationFromParameterServer (ros::NodeHandlePtr &_node_handle,
                                                                              ros::NodeHandlePtr &_private_node_handle,
                                                                              std::string _configuration_namespace) {

        _private_node_handle->param<double>(_configuration_namespace + "weight", weight_, 1.0);
    }


    int CalculatorEstimationBase::returnvalues() {
        return valuescore;
    }

    bool CalculatorEstimationBase::start () {

        ROS_INFO("Started"); 
                                           
        if(!this->run())
        {return false;};
        return true;

    }

    int CalculatorEstimationBase::getresult() {
        return operation_result;
    }


}
