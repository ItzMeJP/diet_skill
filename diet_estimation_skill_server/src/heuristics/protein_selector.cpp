#include <diet_estimation_skill_server/heuristics/protein_selector.h>


namespace diet_estimation_skill {

    /// <summary>
    /// Contructor
    /// </summary>
    ProteinSelector::ProteinSelector() {}

    /// <summary>
    /// Destructor
    /// </summary>
    ProteinSelector::~ProteinSelector() {}


    void ProteinSelector::setupMethodConfigurationFromParameterServer(ros::NodeHandlePtr &_node_handle,ros::NodeHandlePtr &_private_node_handle, std::string _configuration_namespace){

        _private_node_handle->param<double>(_configuration_namespace + "threshold", threshold_, 1.0);
        this->setupBaseConfigurationFromParameterServer(_node_handle, _private_node_handle, _configuration_namespace);
    }

    bool ProteinSelector::run(){

        protein_value_arr_.clear();
        previous_scores_arr_.clear();
        resulted_scores_arr_.clear();

        ROS_INFO_STREAM("Running protein selector");

        int i = 0;

        std::vector<CandidateTuple>::iterator it;
        for (it = candidate_list_.begin(); it != candidate_list_.end(); ++it)
        {
            extrapolation_arr_.push_back(std::get<1>(*it));
            previous_scores_arr_.push_back(std::get<2>(*it));

            if( !std::get<1>(*it) ) {
                protein_value_arr_.push_back((double) std::get<0>(*it).protein_rate * std::get<0>(*it).gram);
                if(protein_value_arr_.back() < threshold_) {
                    extrapolation_arr_.back() = true;
                    std::get<1>(*it) = true;
                    protein_value_arr_.back() = 0;
                }
            }
            else
                protein_value_arr_.push_back(0);

            i++;
        }

        if(std::accumulate(extrapolation_arr_.begin(), extrapolation_arr_.end(), 0) == (int)protein_value_arr_.size()){

            ROS_ERROR_STREAM("All candidates extrapolate the Protein threshold.");
            return false;
        }

        if(!std_vector_operations::normalizeVector(protein_value_arr_))
        {
            ROS_DEBUG_STREAM("Max == Min. All candidates are eligible (before extrapolation analysis) according to the protein selector array.");
            std::vector<double>::iterator dit;
            for (dit = protein_value_arr_.begin(); dit != protein_value_arr_.end(); ++dit){
                *dit = 0;
            }
        }

        std_vector_operations::scalarMultiplicationToVector(protein_value_arr_, (weight_)); //penalty weight
        std_vector_operations::sumEachElementOfVector(protein_value_arr_, previous_scores_arr_, resulted_scores_arr_);

        if(!std_vector_operations::normalizeVector(resulted_scores_arr_))
        {
            ROS_DEBUG_STREAM("Max == Min. All candidates are eligible (before extrapolation analysis) according to the protein selector array.");
            std::vector<double>::iterator dit;
            for (dit = resulted_scores_arr_.begin(); dit != resulted_scores_arr_.end(); ++dit){
                *dit = 0;
            }
        }

        int j=0;
        for (it = candidate_list_.begin(); it != candidate_list_.end(); ++it)
        {

            if( !std::get<1>(*it) )
                std::get<2>(*it) = resulted_scores_arr_.at(j);

            j++;
            ROS_DEBUG_STREAM("Candidate "<<std::get<0>(*it).food_name << " extrapolates? : " << std::get<1>(*it) << ". Score: " << std::get<2>(*it));
        }

        return true;
    }

}
