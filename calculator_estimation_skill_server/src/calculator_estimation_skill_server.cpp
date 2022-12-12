#include <calculator_estimation_skill_server/calculator_estimation_skill_server.h>

namespace grasp_estimation_skill {

/// <summary>
    /// Contructor
    /// </summary>
    GraspEstimationSkill::GraspEstimationSkill () {
        tf_buffer_       = std::make_shared<tf2_ros::Buffer>(ros::Duration(600));
        tf_listener_ptr_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    }

    /// <summary>
    /// Destructor
    /// </summary>
    GraspEstimationSkill::~GraspEstimationSkill () {
        log_data_arr_.clear();
    }

    /// <summary>
    /// Start the skill activating the goal callback
    /// </summary>
    void GraspEstimationSkill::start () {

        actionServer_ = std::make_shared<GraspEstimationSkillActionServer>(*node_handle_, action_server_name_,
                                                                           boost::bind(&GraspEstimationSkill::executeCB,
                                                                                       this, _1));
        actionServer_->start();

    }

    /// <summary>
    /// Goal's callback
    /// </summary>
    /// <param name="goal"> Grasp Estimation Goal (ros action goal).</param>
    void GraspEstimationSkill::executeCB (const grasp_estimation_skill_msgs::GraspEstimationSkillGoalConstPtr &goal) {

        bool name_error = false;

        ROS_INFO_STREAM("Requested object name : " << goal->detected_object_name.c_str());
        ROS_INFO_STREAM("Requested object tf name : " << goal->detected_object_tf_name.c_str());
        detected_object_name_    = goal->detected_object_name.c_str();
        detected_object_tf_name_ = goal->detected_object_tf_name.c_str();

        if (goal->operation_mode != OPERATION_MODE::DIRECT) {
            if (goal->operation_mode == OPERATION_MODE::PRE_LOAD) { //if load operation mode

                estimationPipelineArrPtr_->clear();
                grasp_candidates_arr_.candidates.clear();
                pipeline_extrapolation_arr_.clear();
                candidates_tf_names_arr_.clear();
                pipeline_scores_arr_.clear();
                pipeline_scores_pair_arr_.clear();
                collisions_marker_arr_.markers.clear();

                occur_collision_filter_ = false;
                occur_joint_filter_     = false;
                occur_ws_filter_        = false;

                candidate_chosen_ = "NOT_CONVERGED";

                detected_object_name_loaded_    = detected_object_name_;
                detected_object_tf_name_loaded_ = detected_object_tf_name_;
            } else {

                if ((detected_object_name_loaded_.compare(detected_object_name_) != 0) ||
                    (detected_object_tf_name_loaded_.compare(detected_object_tf_name_) != 0)) {
                    ROS_ERROR_STREAM(
                            "Detected object and/or TF name(s) are different from the ones defined by load option");
                    name_error = true;
                }

            }
        } else {
            estimationPipelineArrPtr_->clear();
            grasp_candidates_arr_.candidates.clear();
            pipeline_extrapolation_arr_.clear();
            candidates_tf_names_arr_.clear();
            pipeline_scores_arr_.clear();
            pipeline_scores_pair_arr_.clear();
            collisions_marker_arr_.markers.clear();

            occur_collision_filter_ = false;
            occur_joint_filter_     = false;
            occur_ws_filter_        = false;

            candidate_chosen_ = "NOT_CONVERGED";
        }

        ros::Time t1, t2;
        t1 = ros::Time::now();

        if (name_error)
            setAborted();
        else
            (executeProcess(goal->operation_mode)) ? setSucceeded() : setAborted(); //I can't set aborted before

        t2 = ros::Time::now();

        if (goal->operation_mode == OPERATION_MODE::DIRECT || goal->operation_mode == OPERATION_MODE::STANDLONE_RUN)
            ROS_WARN_STREAM("Decision Elapsed Time :" << t2 - t1);
        else
            ROS_WARN_STREAM("Load Elapsed Time :" << t2 - t1);

        LogData l;
        l.decision_time    = t2 - t1;
        l.detected_object  = detected_object_name_;
        l.candidate_chosen = candidate_chosen_;

        log_data_arr_.push_back(l);

    }

    /// <summary>
    /// Set skill successed state. Export the selected grasp solution
    /// </summary>
    /// <param name="outcome"> message.</param>
    void GraspEstimationSkill::setSucceeded (std::string outcome) {
        result_.percentage  = 100;
        result_.skillStatus = action_server_name_;
        result_.skillStatus += ": Succeeded";
        result_.grasp_pose  = msg_result_candidate_;
        result_.outcome     = outcome;
        ROS_INFO_STREAM(action_server_name_ << ": Succeeded");
        actionServer_->setSucceeded(result_);
    }

    /// <summary>
    /// Set skill aborted state. Export a null result
    /// </summary>
    /// <param name="outputStr">string in which the translated statement will be added.</param>
    void GraspEstimationSkill::setAborted (std::string outcome) {
        grasp_estimation_skill_msgs::GraspCandidate null_msg;
        result_.percentage  = 0;
        result_.skillStatus = action_server_name_;
        result_.skillStatus += ": Aborted";
        result_.grasp_pose  = null_msg;
        result_.outcome     = outcome;
        ROS_INFO_STREAM(action_server_name_ << ": Aborted");
        actionServer_->setAborted(result_);
    }

    /// <summary>
    /// Action feedback setup
    /// </summary>
    /// <param name="outputStr">string in which the translated statement will be added.</param>
    /// <param name="statement">digital write statement to translate.</param>
    /// <param name="indentation">number of spaces to write on the left of the statement.</param>
    /// <returns>true if succeded, false otherwise.</returns>
    void GraspEstimationSkill::feedback (float percentage) {
        feedback_.percentage  = percentage;
        feedback_.skillStatus = action_server_name_;
        feedback_.skillStatus += " Executing";
        ROS_INFO_STREAM(action_server_name_ << ": Executing. Percentage" << percentage);
        actionServer_->publishFeedback(feedback_);
    }

    /// <summary>
    /// Action's preempt setup
    /// </summary>
    /// <param name="outputStr">string in which the translated statement will be added.</param>
    /// <param name="statement">digital write statement to translate.</param>
    /// <param name="indentation">number of spaces to write on the left of the statement.</param>
    /// <returns>true if succeded, false otherwise.</returns>
    bool GraspEstimationSkill::checkPreemption () {
        if (actionServer_->isPreemptRequested() || !ros::ok()) {
            result_.percentage  = 0;
            result_.skillStatus = action_server_name_;
            result_.skillStatus += ": Preempted";
            result_.outcome     = "preempted";
            ROS_INFO_STREAM(action_server_name_ << ": Preempted");
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
    bool GraspEstimationSkill::setupSkillConfigurationFromParameterServer (ros::NodeHandlePtr &_node_handle,
                                                                           ros::NodeHandlePtr &_private_node_handle) {
        this->node_handle_         = _node_handle;
        this->private_node_handle_ = _private_node_handle;

        package_path_ = ros::package::getPath("grasp_estimation_skill_server");
        logs_path_    = package_path_ + "/logs";

        private_node_handle_->param<std::string>("action_server_name", action_server_name_, "GraspEstimationSkill");

        private_node_handle_->param<std::string>("grasps_candidates_namespace", grasps_candidates_namespace_,
                                                 "candidate");
        private_node_handle_->param<std::string>("center_of_gravity_namespace", cog_namespace_, "center_of_gravity");
        private_node_handle_->param<std::string>("center_of_gravity_namespace_tf_header_frame_id_override",
                                                 cog_namespace_tf_header_frame_id_override_, "gripper");
        private_node_handle_->param<std::string>("center_of_bounding_box_namespace", cobb_namespace_,
                                                 "center_of_bounding_box");
        private_node_handle_->param<std::string>("center_of_bounding_box_namespace_tf_header_frame_id_override",
                                                 cobb_namespace_tf_header_frame_id_override_, "gripper");

        private_node_handle_->param<std::string>("approach/origin_namespace", approach_origin_namespace_, "approach");

        //private_node_handle_->param<std::string>("object_origin_namespace", object_origin_namespace_, "origin");
        //private_node_handle_->param<std::string>("detected_object_namespace", detected_object_namespace_, "detected");
        private_node_handle_->param<std::string>("robot_base_frame", robot_base_frame_, "base_link");
        private_node_handle_->param<std::string>("reference_frame", reference_name_, "gripper");
        private_node_handle_->param<std::string>("estimated_grasp_tf_frame", estimated_grasp_tf_frame_, "grasp");
        //private_node_handle_->param<std::string>("tf_actuation_reference", tf_actuation_reference_, "origin");
        private_node_handle_->param<bool>("load_from_yaml", load_from_yaml_, true);
        private_node_handle_->param<bool>("debug_tools", debug_tools_, true);

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

    bool GraspEstimationSkill::executeProcess (int _operation_mode) {

            if (_operation_mode == OPERATION_MODE::DIRECT){ //Load and Run the Pipeline
                return (executeDirectProcess());
            }
            else if (_operation_mode == OPERATION_MODE::PRE_LOAD) { // Just Load
                return (executePreLoad());
            }
            else if (_operation_mode == OPERATION_MODE::STANDLONE_RUN && preload_ok_) { // Just Run the Pipeline
                preload_ok_ = false;
                return (executeStandloneProcess());
            }
            else if (!preload_ok_) {
                ROS_ERROR_STREAM("No loaded data found");
                return false;
            }
            return true;

    }

    /// <summary>
    /// Execute the load and run mode together
    /// </summary>
    /// <returns>true if succeeded, otherwise false </returns>
    bool GraspEstimationSkill::executeDirectProcess () {

        //start the procedure after goal acquisition.     It loads parameter server and run the procedure START
        if (!getNumberOfCandidates()
            || !readData()
            || !readCOGData()
            || !pubCogTF()
            || !readCoBBData()
            || !pubCobbTF()
            || !pubCandidates()
            || !loadEstimationPipeline()
            || !buildTFsArr()
            || !runMethods()
            || !sortScores()
            || !getResult()
            || !pubChoosenCandidate()
                ){
                    if (debug_tools_) {
                        runDebugTools(); // only pub debug tools that are not related to a candidate
                    }

                    return false;
                }

        return true;
    }

    /// <summary>
    /// Execute the load mode
    /// </summary>
    /// <returns>true if succeeded, otherwise false </returns>
    bool GraspEstimationSkill::executePreLoad () {

        if (!getNumberOfCandidates()
            || !readData()
            || !readCOGData()
            || !pubCogTF()
            || !readCoBBData()
            || !pubCobbTF()
            || !pubCandidates()
            || !loadEstimationPipeline()
            || !buildTFsArr())
        {return false;}

        preload_ok_ = true;
        return true;

    }

    /// <summary>
    /// Execute the standalone run mode
    /// </summary>
    /// <returns>true if succeeded, otherwise false </returns>
    bool GraspEstimationSkill::executeStandloneProcess () {
        if (!runMethods()
            || !sortScores()
            || !getResult()
            || !pubChoosenCandidate()){

            if (debug_tools_) {
                runDebugTools(); // only pub debug tools that are not related to a candidate
            }

            return false;
        }

        return true;
    }

    /// <summary>
    /// Check if the input string exists in ROS parameter server
    /// </summary>
    /// <param name="_input mode"> Input string to be assessmented </param>
    /// <returns>true if the string exists, otherwise false </returns>
    bool GraspEstimationSkill::checkNamespace (std::string _input) {
        std::string s;
        if (!private_node_handle_->searchParam(_input, s)) {
            ROS_ERROR_STREAM("The string (" << _input << ") does not exist in the parameter server tree. "
                                                         "The number of candidates in the configuration file of ("
                                            << detected_object_name_ << ") is less than " <<
                                            number_of_candidates_ << " (\"number_of_candidates\" parameter value).");
            return false;
        } else
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
    bool GraspEstimationSkill::loadEstimationPipeline () {

        //just to know the order of the pipeline metrics
        XmlRpc::XmlRpcValue xml_param;
        std::string         configuration_namespace =
                                    private_node_handle_->getNamespace() + "/" + detected_object_name_ + "/pipeline";
        ROS_DEBUG_STREAM("Config_ns to load pipelines: " << configuration_namespace);
        if (private_node_handle_->getParam(configuration_namespace, xml_param) &&
            xml_param.getType() == XmlRpc::XmlRpcValue::TypeStruct) {

            //ROS_DEBUG_STREAM("xml_param: " << xml_param);

            GraspEstimationBase::Ptr           method;
            //int i = 1;
            for (XmlRpc::XmlRpcValue::iterator it = xml_param.begin(); it != xml_param.end(); ++it) {
                std::string method_name = it->first;

                if (method_name.find("euclidean") != std::string::npos) {
                    method.reset(new EuclideanGraspEstimation());
                } else if (method_name.find("depth") != std::string::npos) {
                    method.reset(new DepthGraspEstimation());
                } else if (method_name.find("roll") != std::string::npos) {
                    method.reset(new RollGraspEstimation());
                } else if (method_name.find("pitch") != std::string::npos) {
                    method.reset(new PitchGraspEstimation());
                } else if (method_name.find("yaw") != std::string::npos) {
                    method.reset(new YawGraspEstimation());
                } else if (method_name.find("joints") != std::string::npos) {
                    method.reset(new JointsGraspEstimation());
                } else if (method_name.find("workspace") != std::string::npos) {
                    method.reset(new ConeWSGraspEstimation());
                } else if (method_name.find("collision") != std::string::npos) {
                    method.reset(new CollisionGraspEstimation());
                } else if (method_name.find("cog_distance") != std::string::npos) {
                    method.reset(new COGDistanceGraspEstimation());
                } else if (method_name.find("cobb_distance") != std::string::npos) {
                    method.reset(new COBBDistanceGraspEstimation());
                }

                //TODO: insert new methods ...

                if (method) {
                    ROS_INFO_STREAM("Loading parameters for : " << configuration_namespace + "/" + method_name);
                    method->setTfBuffer(tf_buffer_);
                    method->setupMethodConfigurationFromParameterServer(node_handle_, private_node_handle_,
                                                                        configuration_namespace + "/" + method_name +
                                                                        "/");
                    estimationPipelineArrPtr_->push_back(method);
                }
            }

        }
        if (estimationPipelineArrPtr_->empty()) {
            ROS_ERROR_STREAM("The grasp estimation pipeline is empty");
            return false;
        }

        return true;
    }

    bool GraspEstimationSkill::buildTFsArr () {

        tfs_arr_.child_from_reference_frame_arr.clear();

        tfs_arr_.child_from_robot_frame_arr.clear();

        tfs_arr_.child_from_cog_frame_arr.clear();
        geometry_msgs::TransformStamped child_from_cog_frame;

        tfs_arr_.child_from_cobb_frame_arr.clear();
        geometry_msgs::TransformStamped child_from_cobb_frame;


        // TODO: Correct this issue in other way <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
        // I did that because in the first incidence the erro happens (i run it again and nothign happens)
//        try{
//            child_from_estimated_obj_tf = tf_buffer_->lookupTransform(tf_candidates_name_arr_.at(0), tf_base_reference_name_, ros::Time(0), ros::Duration(0.5));
//                    }
//        catch (tf2::TransformException ex){
//            ROS_ERROR("%s",ex.what());
//        }
        // TODO: Correct this issue in other way <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<

        ROS_DEBUG("Building COG and COBB TFs array...");

        for (size_t i = 0; i < candidates_tf_names_arr_.size(); ++i) {

            try{
                child_from_cog_frame = tf_buffer_->lookupTransform(cog_.child_frame_id , candidates_tf_names_arr_.at(i), ros::Time(0), ros::Duration(0.5));
                tfs_arr_.child_from_cog_frame_arr.push_back(child_from_cog_frame);
            }
            catch (tf2::TransformException ex){
                ROS_ERROR("%s",ex.what());
                return false;
            }

            try{
                child_from_cobb_frame = tf_buffer_->lookupTransform(center_of_bounding_box_.child_frame_id, candidates_tf_names_arr_.at(i), ros::Time(0), ros::Duration(0.5));
                tfs_arr_.child_from_cobb_frame_arr.push_back(child_from_cobb_frame);
            }
            catch (tf2::TransformException ex){
                ROS_ERROR("%s",ex.what());
                return false;
            }

        }

        return true;
    }

    /// <summary>
    /// Run a method defined by GraspEstimationBase class and accumulate the master score and extrapolation arrays
    /// </summary>
    /// <returns> false if :
    /// * the method failed to be executed;
    /// * issues related  to scores sum
    /// * all candidates exceed the restrictions of the methods pipeline (accumulated)
    /// * cannot get additional data from method in debug_tools mode
    /// </returns>
    bool GraspEstimationSkill::runMethods () {

        std::vector<double> pipeline_scores_arr_b;
        std::string         debug_string_ext = "Pipeline extrapolation array:\n",
                            debug_string_sco = "Pipeline score array:\n";

        int lol = estimationPipelineArrPtr_->size();

        for (size_t i = 0; i < estimationPipelineArrPtr_->size(); ++i) {


            if (!estimationPipelineArrPtr_->at(i)->start(robot_base_frame_, reference_name_,
                                                         candidates_tf_names_arr_,tfs_arr_, grasp_candidates_arr_,
                                                         pipeline_extrapolation_arr_)) {
                if (debug_tools_) {
                    ROS_INFO_STREAM("Debug tools enable");
                    if (!getAdditionalData(*estimationPipelineArrPtr_->at(i))) { return false; }
                }
                return false;
            }

            if (i == 0) {
                pipeline_extrapolation_arr_ = estimationPipelineArrPtr_->at(i)->getExtrapolationArr();
                pipeline_scores_arr_        = estimationPipelineArrPtr_->at(i)->getWeightedScoreArr();
            } else {
                if (!grasp_estimation_skill::std_vector_operations::sumEachElementOfVector(pipeline_extrapolation_arr_,
                                                                                           estimationPipelineArrPtr_->at(
                                                                                                   i)->getExtrapolationArr(),
                                                                                           pipeline_extrapolation_arr_)) {
                    ROS_ERROR_STREAM("Array sum operation error. Different size arrays");
                    ROS_ERROR_STREAM("Current extrapolation array size:" << pipeline_extrapolation_arr_.size());
                    ROS_ERROR_STREAM("Extrapolation array heuristic index #"<< i << " size:" << estimationPipelineArrPtr_->at(i)->getExtrapolationArr().size());

                    return false;
                }

                pipeline_scores_arr_b = estimationPipelineArrPtr_->at(i)->getWeightedScoreArr();

                if (!grasp_estimation_skill::std_vector_operations::sumEachElementOfVector(pipeline_scores_arr_,
                                                                                           pipeline_scores_arr_b,
                                                                                           pipeline_scores_arr_)) {
                    ROS_ERROR_STREAM("Array sum operation error. Different size arrays");
                    ROS_ERROR_STREAM("Current score array size:" << pipeline_extrapolation_arr_.size());
                    ROS_ERROR_STREAM("Score array heuristic index #"<< i << " size:" << estimationPipelineArrPtr_->at(i)->getExtrapolationArr().size());

                    return false;
                }
            }

            if (debug_tools_) {
                ROS_INFO_STREAM("Debug tools enable");
                if (!getAdditionalData(*estimationPipelineArrPtr_->at(i))) { return false; }
            }

        }

        for (size_t k = 0; k <
                           pipeline_extrapolation_arr_.size(); ++k) { // To guarantee the worst score to a candidate that extrapolates at least one metric
            if (pipeline_extrapolation_arr_.at(k) == 1) {
                pipeline_scores_arr_.at(k) = (double) *std::max_element(pipeline_scores_arr_.begin(),
                                                                        pipeline_scores_arr_.end()); // penalty. The lowest score will be elected
            }

        }


        if (!grasp_estimation_skill::std_vector_operations::normalizeVector(pipeline_scores_arr_)) {
            ROS_DEBUG_STREAM("Max == Min. All candidates are eligible according to the pipeline score array");
            for (size_t j = 0; j < pipeline_scores_arr_.size(); ++j) {
                pipeline_scores_arr_[j] = 0;
            }
        }


        //only to debug
        if (ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug)) {
            for (size_t j = 0; j < pipeline_extrapolation_arr_.size(); ++j) {
                debug_string_ext += "  " + std::to_string(pipeline_extrapolation_arr_.at(j)) + "\n";
                debug_string_sco += "  " + std::to_string(pipeline_scores_arr_.at(j)) + "\n";
            }

            ROS_DEBUG_STREAM(debug_string_ext);
            ROS_DEBUG_STREAM(debug_string_sco);
        }


        if (std::accumulate(pipeline_extrapolation_arr_.begin(), pipeline_extrapolation_arr_.end(), 0) ==
            candidates_tf_names_arr_.size()) {
            ROS_ERROR_STREAM("All candidates extrapolates at least one threshold.");
            return false;
        }


        return true;
    }



    /// <summary>
    /// Load grasping candidates data from parameter server
    /// </summary>
    /// <returns> return false if:
    /// *hierarchy namespace issues
    /// *error reading gripper data namespace
    /// </returns>
    bool GraspEstimationSkill::readDataFromYaml () {

        ROS_INFO_STREAM("Reading data from YAML");
        grasp_estimation_skill_msgs::GraspCandidate candidate;

        int         g;
        std::string current_candidate,
                    base_tree;

        for (int j = 0; j < number_of_candidates_; ++j) {

            candidate.transform_stamped.header.stamp = ros::Time::now(); // TODO: check if any issue can happen with this instruction code position
            candidate.transform_stamped.child_frame_id =
                    detected_object_name_ + "/" + grasps_candidates_namespace_ + "_" + std::to_string(j);

            current_candidate = "candidate_" + std::to_string(j);

            if (!checkNamespace(detected_object_name_ + "/" + current_candidate))
                return false;

            base_tree = detected_object_name_ + "/" + current_candidate;

            candidates_tf_names_arr_.push_back(base_tree);

            std::vector<double> gripper_param_arr;

            private_node_handle_->param(base_tree + "/gripper/type", g, 0);

            // Generic Version
            if (!private_node_handle_->param(base_tree + "/gripper/parameters", gripper_param_arr,
                                             std::vector<double>())) {
                ROS_ERROR_STREAM("Error reading gripper data from " << base_tree);
                return false;
            }

            private_node_handle_->param(base_tree + "/parent_frame_id", candidate.transform_stamped.header.frame_id,
                                        detected_object_tf_name_);

            private_node_handle_->param(base_tree + "/position/x", candidate.transform_stamped.transform.translation.x,
                                        0.0);
            private_node_handle_->param(base_tree + "/position/y", candidate.transform_stamped.transform.translation.y,
                                        0.0);
            private_node_handle_->param(base_tree + "/position/z", candidate.transform_stamped.transform.translation.z,
                                        0.0);
            private_node_handle_->param(base_tree + "/orientation/x", candidate.transform_stamped.transform.rotation.x,
                                        0.0);
            private_node_handle_->param(base_tree + "/orientation/y", candidate.transform_stamped.transform.rotation.y,
                                        0.0);
            private_node_handle_->param(base_tree + "/orientation/z", candidate.transform_stamped.transform.rotation.z,
                                        0.0);
            private_node_handle_->param(base_tree + "/orientation/w", candidate.transform_stamped.transform.rotation.w,
                                        1.0);
            candidate.gripper_data.type       = g;
            candidate.gripper_data.parameters = gripper_param_arr;

            generateApproachData();

            candidate.approach_stamped.header.stamp    = candidate.transform_stamped.header.stamp;
            candidate.approach_stamped.header.frame_id = candidate.transform_stamped.child_frame_id;
            candidate.approach_stamped.child_frame_id  = base_tree + "/" + approach_origin_namespace_;
            candidate.approach_stamped.transform       = approachPose_.transform;

            if (candidate.transform_stamped.header.frame_id.empty())
                candidate.transform_stamped.header.frame_id = detected_object_tf_name_;


            grasp_candidates_arr_.object_name = detected_object_name_;
            grasp_candidates_arr_.candidates.push_back(candidate);


            ROS_DEBUG_STREAM(
                    current_candidate + " added: [x: " << candidate.transform_stamped.transform.translation.x <<
                                                       " |y: " << candidate.transform_stamped.transform.translation.y <<
                                                       " |z: " << candidate.transform_stamped.transform.translation.z <<
                                                       " |rx: " << candidate.transform_stamped.transform.rotation.x <<
                                                       " |ry: " << candidate.transform_stamped.transform.rotation.y <<
                                                       " |rz: " << candidate.transform_stamped.transform.rotation.z <<
                                                       " |rw: " << candidate.transform_stamped.transform.rotation.w
                                                       << "]");
        }

        return true;
    }



    }

}
