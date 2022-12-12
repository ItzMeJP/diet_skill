#include <ros/ros.h>



#ifndef CALCULATOR_ESTIMATION_BASE_H
#define CALCULATOR_ESTIMATION_BASE_H

namespace calculator_estimation_skill {
    class CalculatorEstimationBase {
    public:
        CalculatorEstimationBase();
        ~CalculatorEstimationBase();



        virtual void setupMethodConfigurationFromParameterServer(ros::NodeHandlePtr& _node_handle, ros::NodeHandlePtr& _private_node_handle, std::string _configuration_namespace) = 0;
        void setupBaseConfigurationFromParameterServer(ros::NodeHandlePtr& _node_handle, ros::NodeHandlePtr& _private_node_handle, std::string _configuration_namespace);
    
        bool start (std::string _robot_base_frame, std::string _tf_base_reference_name_,
                    std::vector<std::string> _tf_candidates_name_arr_); //TODO: change all these vector to only candidates_arr

    
    protected:
        virtual bool run() = 0;

        //Specific private method and parameters
        /// Collision
        //std::vector<grasp_estimation_skill_msgs::GeometricShape> collision_shapes_each_candidate_arr_;
        //std::vector<pcl::PointCloud<pcl::PointXYZRGBNormal>> collision_clouds_each_candidate_arr_;
        //int number_of_collision_shapes_per_candidate_;

    };
}
