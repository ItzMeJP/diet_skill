#include <diet_estimation_skill_server/diet_estimation_base.h>

#include <math.h>
#include <angles/angles.h>
#include <numeric>

#ifndef FIBER_SELECTOR_BASE_H
#define FIBER_SELECTOR_BASE_H


namespace diet_estimation_skill {
    class FiberSelector : public DietEstimationBase {
    public:
        //Constructor
        FiberSelector();
        ~FiberSelector();

        void setupMethodConfigurationFromParameterServer(ros::NodeHandlePtr &_node_handle,ros::NodeHandlePtr &_private_node_handle, std::string _configuration_namespace);

    protected:

        bool run();
        double threshold_;
        std::vector<double> protein_value_arr_, previous_scores_arr_, resulted_scores_arr_;
        std::vector<bool> extrapolation_arr_;
    };

}

#endif