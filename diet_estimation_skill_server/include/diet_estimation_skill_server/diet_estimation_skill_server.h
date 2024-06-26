// ROS includes
#include <ros/ros.h>
#include <ros/package.h>
#include <std_msgs/String.h>
#include <actionlib/server/simple_action_server.h>

// Skill msgs includes
#include <diet_estimation_skill_msgs/DietEstimationSkillAction.h>
#include <diet_estimation_skill_msgs/Candidate.h>

// Skill includes
#include <diet_estimation_skill_server/diet_estimation_base.h>
#include <diet_estimation_skill_server/heuristics/protein_selector.h>
#include <diet_estimation_skill_server/heuristics/fiber_selector.h>



// Std includes
#include <memory>
#include <utility>      // std::pair
#include <numeric>      // accumulate


namespace diet_estimation_skill {
    class DietEstimationSkill {
    public:
        typedef actionlib::SimpleActionServer<diet_estimation_skill_msgs::DietEstimationSkillAction> DietEstimationSkillActionServer;
        typedef actionlib::SimpleActionServer<diet_estimation_skill_msgs::DietEstimationSkillAction> TriggerSkillActionServer;

        enum OPERATION_MODE {
            DIRECT,
            PRE_LOAD,
            STANDALONE_RUN
        };

        //Contructor
        DietEstimationSkill ();

        ~DietEstimationSkill (void);


        void start (); //start the server

        void executeCB (
                const diet_estimation_skill_msgs::DietEstimationSkillGoalConstPtr &goal); // recognize the goal request

        void feedback (float percentage);

        void setSucceeded (std::string outcome = "succeeded");

        void setAborted (std::string outcome = "aborted");

        bool checkPreemption ();

        bool setupSkillConfigurationFromParameterServer (ros::NodeHandlePtr &_node_handle,
                                                         ros::NodeHandlePtr &_private_node_handle);




    protected:

        ros::NodeHandlePtr node_handle_;
        ros::NodeHandlePtr private_node_handle_;
        ros::Publisher     pub_candidates_,
                           pub_ws_cloud_;


        std::shared_ptr<DietEstimationSkillActionServer> actionServer_;
        std::string                                       action_server_name_;


        diet_estimation_skill_msgs::DietEstimationSkillFeedback feedback_, trigger_feedback_;
        diet_estimation_skill_msgs::DietEstimationSkillResult   result_, trigger_result_;

        diet_estimation_skill_msgs::Candidate msg_result_candidate_;

        DietEstimationBase::ArrPtr estimationPipelineArrPtr_ = std::make_shared<DietEstimationBase::Arr>();

        DietEstimationBase::CandidateList candidate_list_;

        bool preload_ok_;
        int number_of_candidates_ = 0;
        std::string patient_name_;

        bool loadEstimationPipeline ();
        bool readDataCandidatesFromParameterServer() ;
        bool executeProcess (int _operation_mode);
        bool executeDirectProcess ();
       // bool executePreLoad ();
       // bool executeStandaloneProcess ();
        bool getNumberOfCandidates();
        bool checkNamespace (std::string _input);
        bool runMethods () ;

        bool getResult();






        };
}
