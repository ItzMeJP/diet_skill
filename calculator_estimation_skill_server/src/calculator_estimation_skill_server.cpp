#include <calculator_estimation_skill_server/calculator_estimation_skill_server.h>

namespace calculator_estimation_skill {

/// <summary>
    /// Contructor
    /// </summary>
    CalculatorEstimationSkill::CalculatorEstimationSkill () {}

    /// <summary>
    /// Destructor
    /// </summary>
    CalculatorEstimationSkill::~CalculatorEstimationSkill () {}

    /// <summary>
    /// Start the skill activating the goal callback
    /// </summary>
    void CalculatorEstimationSkill::start () {

        actionServer_ = std::make_shared<CalculatorEstimationSkillActionServer>(*node_handle_, action_server_name_,
                                                                           boost::bind(&CalculatorEstimationSkill::executeCB,
                                                                                       this, _1));
        actionServer_->start();

    }

    /// <summary>
    /// Goal's callback
    /// </summary>
    /// <param name="goal"> Grasp Estimation Goal (ros action goal).</param>
    void CalculatorEstimationSkill::executeCB (const calculator_estimation_skill_msgs::CalculatorEstimationSkillGoalConstPtr &goal) {

        bool name_error = false;

        ROS_INFO_STREAM("Requested operation mode : " << goal->operation_mode);
        ROS_INFO_STREAM("First argument : " << goal->arg1);
        ROS_INFO_STREAM("Second argument : " << goal->arg2);

        executeProcess(goal->operation_mode, goal->arg1,goal->arg2) ? setSucceeded() : setAborted();
    }

    /// <summary>
    /// Set skill successed state. Export the selected grasp solution
    /// </summary>
    /// <param name="outcome"> message.</param>
    void CalculatorEstimationSkill::setSucceeded (std::string outcome) {
        result_.percentage  = 100;
        result_.calc_result = msg_result_candidate_;
        actionServer_->setSucceeded(result_);
    }

    /// <summary>
    /// Set skill aborted state. Export a null result
    /// </summary>
    /// <param name="outputStr">string in which the translated statement will be added.</param>
    void CalculatorEstimationSkill::setAborted (std::string outcome) {
        result_.percentage  = 0;
        result_.calc_result = msg_result_candidate_;
        actionServer_->setAborted(result_);
    }

    /// <summary>
    /// Action feedback setup
    /// </summary>
    /// <param name="outputStr">string in which the translated statement will be added.</param>
    /// <param name="statement">digital write statement to translate.</param>
    /// <param name="indentation">number of spaces to write on the left of the statement.</param>
    /// <returns>true if succeded, false otherwise.</returns>
    void CalculatorEstimationSkill::feedback (float percentage) {
        feedback_.percentage  = percentage;
        actionServer_->publishFeedback(feedback_);
    }

    /// <summary>
    /// Action's preempt setup
    /// </summary>
    /// <param name="outputStr">string in which the translated statement will be added.</param>
    /// <param name="statement">digital write statement to translate.</param>
    /// <param name="indentation">number of spaces to write on the left of the statement.</param>
    /// <returns>true if succeded, false otherwise.</returns>
    bool CalculatorEstimationSkill::checkPreemption () {
        if (actionServer_->isPreemptRequested() || !ros::ok()) {
            result_.percentage  = 0;
            actionServer_->setPreempted(result_);
            return true;
        } else {
            return false;
        }

    }

    /// <summary>
    /// Load all parameters from parameter server regarding the grasp estimation server.
    /// </summary>
    /// <param name="_node_handle">ros node handle</param>
    /// <param name="_private_node_handle">ros private node handle</param>
    /// <returns>true</returns>
    bool CalculatorEstimationSkill::setupSkillConfigurationFromParameterServer (ros::NodeHandlePtr &_node_handle,
                                                                           ros::NodeHandlePtr &_private_node_handle) {
        this->node_handle_         = _node_handle;
        this->private_node_handle_ = _private_node_handle;

        // package_path_ = ros::package::getPath("calculator_estimation_skill_server");

        private_node_handle_->param<std::string>("action_server_name", action_server_name_, "CalculatorEstimationSkill");

        // private_node_handle_->param<std::string>("calculator_candidates_namespace", grasps_candidates_namespace_,
        //                                          "candidate");
        private_node_handle_->param<bool>("load_from_yaml", load_from_yaml_, true);
        return true;

    }

    /// <summary>
    /// Start the grasp estimation pipeline after the goal has been received. Run accord to preload_support_ parameter:
    /// *preload_support_ -> false: the complete pipeline is executed (Load + Run).
    /// *preload_support_ -> true: enable the _operation mode argument.
    /// </summary>
    /// <param name="_operation mode">
    /// Integer representing the operation mode.
    /// * 0 -> Just Load data
    /// * 1 -> Run the pipeline without loading (return error if no previously objected data load is performed.)
    /// </param>
    /// <param name="_private_node_handle">ros private node handle</param>
    /// <returns>true if succeeded, otherwise false </returns>
/*    bool GraspEstimationSkill::executeProcess (int _operation_mode) {

        if (!preload_support_) {
            return (executeDirectProcess());
        } else {

            if (_operation_mode == 1 && preload_ok_) {
                preload_ok_ = false;
                return (executeStandloneProcess());
            } else if (_operation_mode == 0)
                return (executePreLoad());
            else if (!preload_ok_) {
                ROS_ERROR_STREAM("No loaded data found");
                return false;
            }

        }
    }*/

    bool CalculatorEstimationSkill::executeProcess (int _operation_mode, int arg1, int arg2) {

        executeDirectProcess(arg1,arg2);


        return true;

    }

    /// <summary>
    /// Execute the load and run mode together
    /// </summary>
    /// <returns>true if succeeded, otherwise false </returns>
    bool CalculatorEstimationSkill::executeDirectProcess (int arg1, int arg2) {

        //start the procedure after goal acquisition.     It loads parameter server and run the procedure START
        if (!loadEstimationPipeline(arg1,  arg2)
            || !readDataFromYaml ())
        {return false;}    


        return true;
    }



    /// <summary>
    /// Load the object pipeline from parameter server. Supported methods's tag names (X is an alpha_numeric that define the order):
    /// X_euclidean: euclidean distance evaluator
    /// X_depth: depth distance evaluator
    /// X_roll: roll distance evaluator
    /// X_pitch: pitch distance evaluator
    /// X_joints: joint filter
    /// X_workspace: workspace filter
    /// X_collision: collision filter
    /// </summary>
    /// <returns> false if the pipeline is empty (no identified tag), otherwise true </returns>
    bool CalculatorEstimationSkill::loadEstimationPipeline (int arg1, int arg2) {

        //just to know the order of the pipeline metrics
        XmlRpc::XmlRpcValue xml_param;
        std::string         configuration_namespace =
                                    private_node_handle_->getNamespace() + "/" + "operations/pipeline";
        std::cout << configuration_namespace << std::endl;
        ROS_DEBUG_STREAM("Config_ns to load pipelines: " << configuration_namespace);
        if (private_node_handle_->getParam(configuration_namespace, xml_param) &&
            xml_param.getType() == XmlRpc::XmlRpcValue::TypeStruct) {

            //ROS_DEBUG_STREAM("xml_param: " << xml_param);
            std::cout << xml_param << std::endl;

            CalculatorEstimationBase::Ptr           method;


            struct candidateData *p;

            //int i = 1;
            for (XmlRpc::XmlRpcValue::iterator it = xml_param.begin(); it != xml_param.end(); ++it) {
                std::string method_name = it->first;

                std::cout << method_name << std::endl;
                ROS_INFO("Reading yaml file");

                if (method_name.find("sum") != std::string::npos) {
                    ROS_INFO("1");
                    method.reset(new SumEstimation());
                } else if (method_name.find("sub") != std::string::npos) {
                    ROS_INFO("2");
                    method.reset(new SubEstimation());}
                // } else if (method_name.find("roll") != std::string::npos) {
                //     method.reset(new RollGraspEstimation());
                // } else if (method_name.find("pitch") != std::string::npos) {
                //     method.reset(new PitchGraspEstimation());
        //         } else if (method_name.find("yaw") != std::string::npos) {
        //             method.reset(new YawGraspEstimation());
        //         } else if (method_name.find("joints") != std::string::npos) {
        //             method.reset(new JointsGraspEstimation());
        //         } else if (method_name.find("workspace") != std::string::npos) {
        //             method.reset(new ConeWSGraspEstimation());
        //         } else if (method_name.find("collision") != std::string::npos) {
        //             method.reset(new CollisionGraspEstimation());
        //         } else if (method_name.find("cog_distance") != std::string::npos) {
        //             method.reset(new COGDistanceGraspEstimation());
        //         } else if (method_name.find("cobb_distance") != std::string::npos) {
        //             method.reset(new COBBDistanceGraspEstimation());
        //         }

        //         //TODO: insert new methods ...

                if (method) {
                    ROS_INFO_STREAM("Loading parameters for : " << configuration_namespace + "/" + method_name);
                    // method->setTfBuffer(tf_buffer_);
                    method->setupMethodConfigurationFromParameterServer(node_handle_, private_node_handle_,
                                                                        configuration_namespace + "/" + method_name +
                                                                        "/");
                    // estimationPipelineArrPtr_->push_back(method);
                    method->setarguments(arg1, arg2);
                    method->start();
                    std::cout << "Results = " << method->getresult() << std::endl;
                    msg_result_candidate_.operation_result=method->getresult();
                    msg_result_candidate_.operation_rest= 3;
                    // msg_result_candidate_=test1;
                }
            }

        }
        // if (estimationPipelineArrPtr_->empty()) {
        //     ROS_ERROR_STREAM("The grasp estimation pipeline is empty");
        //     return false;
        // }

        return true;
    }


   
    bool CalculatorEstimationSkill::readDataFromYaml () {

        ROS_INFO_STREAM("Reading data from YAML");
        // grasp_estimation_skill_msgs::GraspCandidate candidate;

        // int         g;
        // std::string current_candidate,
        //             base_tree;

        // for (int j = 0; j < number_of_candidates_; ++j) {

        //     candidate.transform_stamped.header.stamp = ros::Time::now(); // TODO: check if any issue can happen with this instruction code position
        //     candidate.transform_stamped.child_frame_id =
        //             detected_object_name_ + "/" + grasps_candidates_namespace_ + "_" + std::to_string(j);

        //     current_candidate = "candidate_" + std::to_string(j);

        //     if (!checkNamespace(detected_object_name_ + "/" + current_candidate))
        //         return false;

        //     base_tree = detected_object_name_ + "/" + current_candidate;

        //     candidates_tf_names_arr_.push_back(base_tree);

        //     std::vector<double> gripper_param_arr;

        //     private_node_handle_->param(base_tree + "/gripper/type", g, 0);

        //     // Generic Version
        //     if (!private_node_handle_->param(base_tree + "/gripper/parameters", gripper_param_arr,
        //                                      std::vector<double>())) {
        //         ROS_ERROR_STREAM("Error reading gripper data from " << base_tree);
        //         return false;
        //     }

        //     private_node_handle_->param(base_tree + "/parent_frame_id", candidate.transform_stamped.header.frame_id,
        //                                 detected_object_tf_name_);

        //     private_node_handle_->param(base_tree + "/position/x", candidate.transform_stamped.transform.translation.x,
        //                                 0.0);
        //     private_node_handle_->param(base_tree + "/position/y", candidate.transform_stamped.transform.translation.y,
        //                                 0.0);
        //     private_node_handle_->param(base_tree + "/position/z", candidate.transform_stamped.transform.translation.z,
        //                                 0.0);
        //     private_node_handle_->param(base_tree + "/orientation/x", candidate.transform_stamped.transform.rotation.x,
        //                                 0.0);
        //     private_node_handle_->param(base_tree + "/orientation/y", candidate.transform_stamped.transform.rotation.y,
        //                                 0.0);
        //     private_node_handle_->param(base_tree + "/orientation/z", candidate.transform_stamped.transform.rotation.z,
        //                                 0.0);
        //     private_node_handle_->param(base_tree + "/orientation/w", candidate.transform_stamped.transform.rotation.w,
        //                                 1.0);
        //     candidate.gripper_data.type       = g;
        //     candidate.gripper_data.parameters = gripper_param_arr;

        //     generateApproachData();

        //     candidate.approach_stamped.header.stamp    = candidate.transform_stamped.header.stamp;
        //     candidate.approach_stamped.header.frame_id = candidate.transform_stamped.child_frame_id;
        //     candidate.approach_stamped.child_frame_id  = base_tree + "/" + approach_origin_namespace_;
        //     candidate.approach_stamped.transform       = approachPose_.transform;

        //     if (candidate.transform_stamped.header.frame_id.empty())
        //         candidate.transform_stamped.header.frame_id = detected_object_tf_name_;


        //     grasp_candidates_arr_.object_name = detected_object_name_;
        //     grasp_candidates_arr_.candidates.push_back(candidate);

        // }

        return true;
    }



    }
