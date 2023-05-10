#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>


#ifndef CALCULATOR_ESTIMATION_BASE_H
#define CALCULATOR_ESTIMATION_BASE_H

namespace calculator_estimation_skill {
    class CalculatorEstimationBase {
    public:
        //Constructor
        CalculatorEstimationBase();
        ~CalculatorEstimationBase();

        typedef std::shared_ptr<CalculatorEstimationBase> Ptr;

        struct candidateData{

            int arg1;
            int arg2;

        };



        virtual void setupMethodConfigurationFromParameterServer(ros::NodeHandlePtr& _node_handle, ros::NodeHandlePtr& _private_node_handle, std::string _configuration_namespace) = 0; //Template

        virtual bool setarguments(int arg1, int arg2) = 0; 


        void setupBaseConfigurationFromParameterServer(ros::NodeHandlePtr& _node_handle, ros::NodeHandlePtr& _private_node_handle, std::string _configuration_namespace);
    
        bool start (); //TODO: change all these vector to only candidates_arr

        int returnvalues();

        int getresult();

    protected:
        //Protected variables
        virtual bool run() = 0;  //Template

        int argument1, argument2, valuescore;
        int operation_result = 0;
        double weight_;
        //Specific private method and parameters
        /// Collision
        //std::vector<grasp_estimation_skill_msgs::GeometricShape> collision_shapes_each_candidate_arr_;
        //std::vector<pcl::PointCloud<pcl::PointXYZRGBNormal>> collision_clouds_each_candidate_arr_;
        //int number_of_collision_shapes_per_candidate_;

    };
}

#endif