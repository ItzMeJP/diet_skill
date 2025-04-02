#include <diet_estimation_skill_server/diet_estimation_base.h>
#include <diet_estimation_skill_server/data/fiber_data.h>

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


        void setData(std::shared_ptr<DietEstimationDataBase> _d) override {
            std::shared_ptr<FiberData> fiberData = std::dynamic_pointer_cast<FiberData>(_d);
            if (fiberData) {
                std::cout << "Valor de foo_fiber: " << fiberData->foo_fiber << std::endl;
            }
        }

    protected:

        bool run();
        double threshold_;
        std::vector<double> protein_value_arr_, previous_scores_arr_, resulted_scores_arr_;
        std::vector<bool> extrapolation_arr_;
    };

}

#endif