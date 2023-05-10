// ROS includes
#include <ros/ros.h>
#include <ros/package.h>
#include <std_msgs/String.h>
#include <actionlib/server/simple_action_server.h>

// Skill msgs includes
#include <calculator_estimation_skill_msgs/CalculatorEstimationSkillAction.h>
#include <calculator_estimation_skill_msgs/CalculatorResult.h>

// Skill includes
#include <calculator_estimation_skill_server/calculator_estimation_base.h>
#include <calculator_estimation_skill_server/sum_estimation.h>
#include <calculator_estimation_skill_server/sub_estimation.h>


// Std includes
#include <memory>
#include <utility>      // std::pair
#include <numeric>      // accumulate


namespace calculator_estimation_skill {
    class CalculatorEstimationSkill {
    public:
        typedef actionlib::SimpleActionServer<calculator_estimation_skill_msgs::CalculatorEstimationSkillAction> CalculatorEstimationSkillActionServer;
        typedef actionlib::SimpleActionServer<calculator_estimation_skill_msgs::CalculatorEstimationSkillAction> TriggerSkillActionServer;

        //Contructor
        CalculatorEstimationSkill ();

        ~CalculatorEstimationSkill (void);


        void start (); //start the server

        void executeCB (
                const calculator_estimation_skill_msgs::CalculatorEstimationSkillGoalConstPtr &goal); // recognize the goal request

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

        struct Tes {
            int a=3;
            int b=4;
        };


        std::shared_ptr<CalculatorEstimationSkillActionServer> actionServer_;
        std::string                                       action_server_name_;


        calculator_estimation_skill_msgs::CalculatorEstimationSkillFeedback feedback_, trigger_feedback_;
        calculator_estimation_skill_msgs::CalculatorEstimationSkillResult   result_, trigger_result_;

        calculator_estimation_skill_msgs::CalculatorResult msg_result_candidate_;

        calculator_estimation_skill::CalculatorEstimationBase::candidateData candidate_Data_;

        bool load_from_yaml_;
        std::string package_path_;


        std::string grasps_candidates_namespace_;


        bool loadEstimationPipeline (int arg1, int arg2);
        bool executeProcess (int _operation_mode, int arg1, int arg2);
        bool executeDirectProcess (int arg1, int arg2);
        bool readDataFromYaml ();



    };
}
