#include <calculator_estimation_skill_server/calculator_estimation_base.h>


namespace calculator_estimation_skill {

    CalculatorEstimationBase::CalculatorEstimationBase(){};

    CalculatorEstimationBase::~CalculatorEstimationBase(){};



    void CalculatorEstimationBase::setupBaseConfigurationFromParameterServer (ros::NodeHandlePtr &_node_handle,
                                                                         ros::NodeHandlePtr &_private_node_handle,
                                                                         std::string _configuration_namespace) {

        _private_node_handle->param<double>(_configuration_namespace + "weight", weight_, 1.0);
    }





    bool CalculatorEstimationBase::start (std::string _robot_base_frame, std::string _tf_base_reference_name_,
                                     std::vector<std::string> _tf_candidates_name_arr_) {

        ROS_INFO("t");                                    
        // if(   !getTFsFromRobotBase()
        //       || !getTFsFromReference()
        //       || !this->run()
        //         ){return false;};
        // return true;
    }


}
