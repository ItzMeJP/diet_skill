#include <ros/ros.h>
#include <unordered_map>
#include <vector>
#include <iostream>
#include <memory>



#include <diet_estimation_skill_msgs/Candidate.h>

#include <diet_estimation_skill_server/common/tuple_operations.h>
#include <diet_estimation_skill_server/common/std_vector_operations.h>

#ifndef DIET_ESTIMATION_BASE_H
#define DIET_ESTIMATION_BASE_H

namespace diet_estimation_skill {
    class DietEstimationBase {
    public:

        typedef std::tuple<diet_estimation_skill_msgs::Candidate,bool, double> CandidateTuple; //Candidate data | Enable Mask | Score
        typedef std::vector<CandidateTuple> CandidateList; // candidate name, filter mask, cost
        typedef std::shared_ptr<DietEstimationBase> Ptr;
        typedef std::vector<DietEstimationBase::Ptr> Arr;
        typedef std::shared_ptr<DietEstimationBase::Arr> ArrPtr;


        //typedef std::unordered_map <std::string, diet_estimation_skill_msgs::Candidate > FoodCandidates;

        //Constructor
        DietEstimationBase();

        //Destructor
        virtual ~DietEstimationBase() = default;

        void setupBaseConfigurationFromParameterServer(ros::NodeHandlePtr& _node_handle, ros::NodeHandlePtr& _private_node_handle, std::string _configuration_namespace);

        virtual void setupMethodConfigurationFromParameterServer(ros::NodeHandlePtr& _node_handle, ros::NodeHandlePtr& _private_node_handle, std::string _configuration_namespace) = 0;
        virtual bool run() = 0;

        CandidateList getResultList();

        void setInitialList(CandidateList _input_list);

        //void setFoodCandidates(FoodCandidates _input_candidates);

    protected:

        double weight_;

        CandidateList candidate_list_;
        //FoodCandidates candidates_list_;



        ros::NodeHandlePtr node_handle_,
                           private_node_handle_;




    };
}

#endif