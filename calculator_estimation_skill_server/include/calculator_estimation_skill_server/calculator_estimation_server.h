// ROS includes
#include <ros/ros.h>
#include <ros/package.h>
#include <std_msgs/String.h>
#include <actionlib/server/simple_action_server.h>

// Skill msgs includes
#include <calculator_estimation_skill_msgs/CalculatorEstimationSkillAction.h>

// Skill includes
#include <calculator_estimation_skill_server/calculator_estimation_base.h>


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

        enum OPERATION_MODE {
            DIRECT,
            PRE_LOAD,
            STANDLONE_RUN
        };

        void start (); //start the server

        void executeCB (
                const grasp_estimation_skill_msgs::GraspEstimationSkillGoalConstPtr &goal); // recognize the goal request

        void feedback (float percentage);

        void setSucceeded (std::string outcome = "succeeded");

        void setAborted (std::string outcome = "aborted");

        bool checkPreemption ();

        bool setupSkillConfigurationFromParameterServer (ros::NodeHandlePtr &_node_handle,
                                                         ros::NodeHandlePtr &_private_node_handle);

        bool generateLog (); //only use when server shutdown

        struct LogData {
            std::string   detected_object;
            //TODO: insert object pose
            std::string   candidate_chosen;
            ros::Duration decision_time;
        };

        std::vector<LogData> log_data_arr_;


    protected:

        ros::NodeHandlePtr node_handle_;
        ros::NodeHandlePtr private_node_handle_;
        ros::Publisher     pub_candidates_,
                           pub_choosen_candidate_,
                           pub_collision_vis_,
                           pub_collision_cloud_,
                           pub_ws_cloud_;


        std::string detected_object_name_, // name of the detected object
                    detected_object_namespace_;



    };
}
