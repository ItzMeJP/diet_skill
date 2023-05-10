#include <calculator_estimation_skill_server/sub_estimation.h>

namespace calculator_estimation_skill{

    SubEstimation::SubEstimation () {}
    SubEstimation::~SubEstimation () {}

    void SubEstimation::setupMethodConfigurationFromParameterServer(ros::NodeHandlePtr &_node_handle,
                                                                    ros::NodeHandlePtr &_private_node_handle,
                                                                    std::string _configuration_namespace) {
       valuescore = 0;
       ROS_INFO("SUB_ESTIMATION");                                                                     
       _private_node_handle->param<double>(_configuration_namespace + "min_arg", min_arg, 0);                                                                     
       this->setupBaseConfigurationFromParameterServer(_node_handle, _private_node_handle, _configuration_namespace);                                                                 
    }
    
    bool SubEstimation::setarguments(int arg1, int arg2){
        argument1=arg1;
        argument2=arg2;

        std::cout << "Sum arguments are: " << argument1 << "and" << argument2 << std::endl;

        operation_result= argument1 - argument2;

        valuescore = argument1 - argument2;

        return true;
    }


    bool SubEstimation::run(){
        ROS_INFO_STREAM("Running subtraction estimation method");
        float arg1, arg2;
        float sub= arg1 - arg2;
        // subtraction_array.clear();

        // subtraction_array.push_back(sub);
        ROS_INFO("Runing calculator sub method");
        return true;
    }
}