#include <calculator_estimation_skill_server/calculator_estimation_base.h>

#include <math.h>
#include <angles/angles.h>
#include <numeric>

#ifndef SUM_ESTIMATION_BASE_H
#define SUM_ESTIMATION_BASE_H

namespace calculator_estimation_skill {
    class SumEstimation : public CalculatorEstimationBase{
    public:
        //Constructor
        SumEstimation();
        ~SumEstimation();
        

        void setupMethodConfigurationFromParameterServer(ros::NodeHandlePtr &_node_handle,ros::NodeHandlePtr &_private_node_handle, std::string _configuration_namespace);

        bool setarguments(int arg1, int arg2);

    protected:
        double min_arg;
        bool run();
    };

}

#endif