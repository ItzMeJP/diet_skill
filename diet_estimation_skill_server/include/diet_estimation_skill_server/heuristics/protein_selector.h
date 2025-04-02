#include <diet_estimation_skill_server/diet_estimation_base.h>
#include <diet_estimation_skill_server/data/protein_data.h>


#include <math.h>
#include <angles/angles.h>
#include <numeric>

#ifndef SUB_ESTIMATION_BASE_H
#define SUB_ESTIMATION_BASE_H


namespace diet_estimation_skill {
    class ProteinSelector : public DietEstimationBase {
    public:
        //Constructor
        ProteinSelector();
        ~ProteinSelector();

        void setupMethodConfigurationFromParameterServer(ros::NodeHandlePtr &_node_handle,ros::NodeHandlePtr &_private_node_handle, std::string _configuration_namespace);

        void setData(std::shared_ptr<DietEstimationDataBase> _d) override {
            std::shared_ptr<ProteinData> proteinData = std::dynamic_pointer_cast<ProteinData>(_d);
            if (proteinData) {
                std::cout << "Valor de foo_protein: " << proteinData->foo_protein << std::endl;
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