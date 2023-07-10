#include <diet_estimation_skill_server/diet_estimation_skill_server.h>

namespace diet_estimation_skill {

    /// <summary>
    /// Contructor
    /// </summary>
    DietEstimationSkill::DietEstimationSkill () {}

    /// <summary>s
    /// Destructor
    /// </summary>
    DietEstimationSkill::~DietEstimationSkill () {}

    /// <summary>
    /// Start the skill activating the goal callback
    /// </summary>
    void DietEstimationSkill::start () {

        actionServer_ = std::make_shared<DietEstimationSkillActionServer>(*node_handle_, action_server_name_,
                                                                           boost::bind(&DietEstimationSkill::executeCB,
                                                                                       this, _1));
        actionServer_->start();

    }

    /// <summary>
    /// Goal's callback
    /// </summary>
    /// <param name="goal"> Diet Estimation Goal (ros action goal).</param>
    void DietEstimationSkill::executeCB (const diet_estimation_skill_msgs::DietEstimationSkillGoalConstPtr &goal) {

        ROS_INFO_STREAM("Requested operation mode : " << goal->operation_mode);
        ROS_INFO_STREAM("Name : " << goal->name);

        patient_name_ = goal->name;

        executeProcess(goal->operation_mode) ? setSucceeded() : setAborted();
    }

    /// <summary>
    /// Set skill succeed state. Export the selected diet solution
    /// </summary>
    /// <param name="outcome"> message.</param>
    void DietEstimationSkill::setSucceeded (std::string outcome) {
        result_.percentage  = 100;
        result_.result = msg_result_candidate_;
        actionServer_->setSucceeded(result_);
    }

    /// <summary>
    /// Set skill aborted state. Export a null result
    /// </summary>
    /// <param name="outputStr">string in which the translated statement will be added.</param>
    void DietEstimationSkill::setAborted (std::string outcome) {
        result_.percentage  = 0;
        result_.result = msg_result_candidate_;
        actionServer_->setAborted(result_);
    }

    /// <summary>
    /// Action feedback setup. Allow to update the feedback msg
    /// </summary>
    /// <param name="percentage"> percentage value.</param>
    void DietEstimationSkill::feedback (float percentage) {
        feedback_.percentage  = percentage;
        actionServer_->publishFeedback(feedback_);
    }

    /// <summary>
    /// Action's preempt setup
    /// </summary>
    /// <returns>true if succeded, false otherwise.</returns>
    bool DietEstimationSkill::checkPreemption () {
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
    bool DietEstimationSkill::setupSkillConfigurationFromParameterServer (ros::NodeHandlePtr &_node_handle,
                                                                           ros::NodeHandlePtr &_private_node_handle) {
        this->node_handle_         = _node_handle;
        this->private_node_handle_ = _private_node_handle;

        private_node_handle_->param<std::string>("action_server_name", action_server_name_, "DietEstimationSkill");

        return true;

    }

    /// <summary>
    /// Start the diet estimation pipeline after the goal has been received.
    /// </summary>
    /// <param name="_operation mode">
    /// <returns>true if succeeded, otherwise false </returns>
    bool DietEstimationSkill::executeProcess (int _operation_mode) {

        feedback (0);
        if (_operation_mode == OPERATION_MODE::DIRECT){ //Load and Run the Pipeline
            return (executeDirectProcess());
        }

        return true;

    }

    /// <summary>
    /// Execute the load and run mode together
    /// </summary>
    /// <returns>true if succeeded, otherwise false </returns>
    bool DietEstimationSkill::executeDirectProcess () {

        if (     !getNumberOfCandidates()
            ||   !loadEstimationPipeline()
            ||   !readDataCandidatesFromParameterServer ()
            ||   !runMethods()
            ||   !getResult()
            )
        {return false;}    


        return true;
    }

    /// <summary>
    /// Load the processing pipeline
    /// </summary>
    /// <returns>true if succeeded to load the pipeline, otherwise false </returns>
    bool DietEstimationSkill::loadEstimationPipeline () {

        ROS_INFO_STREAM("Reading pipeline from parameter server");

        estimationPipelineArrPtr_->clear();

        //just to know the order of the pipeline metrics
        XmlRpc::XmlRpcValue xml_param;
        std::string         configuration_namespace = private_node_handle_->getNamespace()  + "/" + patient_name_ + "/pipeline";
        ROS_DEBUG_STREAM("Config_ns to load pipelines: " << configuration_namespace<< " ");
        if (private_node_handle_->getParam(configuration_namespace, xml_param) &&
            xml_param.getType() == XmlRpc::XmlRpcValue::TypeStruct) {

            //ROS_DEBUG_STREAM("xml_param: " << xml_param);

            DietEstimationBase::Ptr           method;
            std::string delimiter = "_", pre_delimiter;
            size_t pos = 0;
            int current_it = 0;
            for (XmlRpc::XmlRpcValue::iterator it = xml_param.begin(); it != xml_param.end(); ++it) {
                std::string method_name = it->first;

                pos = method_name.find(delimiter);
                pre_delimiter = method_name.substr(0,pos);
                method_name.erase(0, pos + delimiter.length());

                if (method_name=="protein_scorer") {
                    method.reset(new ProteinSelector());
                } else if (method_name == "fiber_scorer") {
                    method.reset(new FiberSelector());
                }
                else{
                    ROS_ERROR_STREAM("Method name " << method_name << " is not supported by the server.");
                }

                method_name = pre_delimiter + delimiter + method_name;

                if (method) {
                    ROS_INFO_STREAM("Loading parameters for : " << configuration_namespace + "/" + method_name);
                    method->setupMethodConfigurationFromParameterServer(node_handle_, private_node_handle_,
                                                                        configuration_namespace + "/" + method_name +
                                                                        "/");
                    estimationPipelineArrPtr_->push_back(method);
                }
                current_it++;
            }

        }
        if (estimationPipelineArrPtr_->empty()) {
            ROS_ERROR_STREAM("The diet estimation pipeline is empty.");
            return false;
        }
        feedback (20);
        return true;
    }

    /// <summary>
    /// Get the total number of food candidates into the ROSPARAM server
    /// </summary>
    /// <returns> return false if:
    /// *None grasping candidate is found
    /// </returns>
    bool DietEstimationSkill::getNumberOfCandidates () {

        number_of_candidates_                       = 0;
        XmlRpc::XmlRpcValue xml_param;
        std::string         configuration_namespace =
                private_node_handle_->getNamespace() + "/" + patient_name_;
        ROS_DEBUG_STREAM("Config_ns to load pipelines: " << configuration_namespace<< " ");
        if (private_node_handle_->getParam(configuration_namespace, xml_param) &&
            xml_param.getType() == XmlRpc::XmlRpcValue::TypeStruct) {

            for (XmlRpc::XmlRpcValue::iterator it = xml_param.begin(); it != xml_param.end(); ++it) {
                std::string method_name = it->first;
                if (method_name.find("candidate_") != std::string::npos) {
                    number_of_candidates_++;
                }
            }
        }

        if (number_of_candidates_ == 0) {
            ROS_ERROR_STREAM("Cannot find candidates for " << patient_name_);
            return false;
        }

        ROS_INFO_STREAM("Number of detected grasp candidates: " << number_of_candidates_);
        feedback (10);
        return true;
    }

    /// <summary>
    /// Get food candidates from the ROSPARAM server
    /// </summary>
    /// <returns> return false if:
    /// *None diet candidate information is found
    /// </returns>
    bool DietEstimationSkill::readDataCandidatesFromParameterServer(){

        ROS_INFO_STREAM("Reading candidates from parameter server.");

        candidate_list_.clear();

        diet_estimation_skill_msgs::Candidate candidate;
        DietEstimationBase::CandidateTuple candidate_tuple;

        std::string current_candidate,
                base_tree, aux;

        for (int j = 0; j < number_of_candidates_; ++j) {

            current_candidate = "candidate_" + std::to_string(j);

            if (!checkNamespace(patient_name_ + "/" + current_candidate))
                return false;

            base_tree = patient_name_ + "/" + current_candidate;


            private_node_handle_->param<std::string>(base_tree + "/food_name", aux , "");
            candidate.food_name = aux;
            private_node_handle_->param(base_tree + "/gram", candidate.gram,1.0);
            private_node_handle_->param(base_tree + "/protein_rate", candidate.protein_rate,1.0);
            private_node_handle_->param(base_tree + "/fiber_rate", candidate.fiber_rate,1.0);



            candidate_tuple = std::make_tuple(candidate,false,0); //candidate, extrapolation mask, score

            candidate_list_.push_back(candidate_tuple);

            ROS_DEBUG_STREAM(
                    current_candidate + " added: [Name: " << candidate.food_name <<
                                        " \n: Protein rate: " << candidate.protein_rate <<
                                        " \n: Fiber Rate" << candidate.fiber_rate << "]");
        }

        feedback (30);
        return true;

    }

    /// <summary>
    /// Check if the input string exists in ROS parameter server
    /// </summary>
    /// <param name="_input mode"> Input string to be assessmented </param>
    /// <returns>true if the string exists, otherwise false </returns>
    bool DietEstimationSkill::checkNamespace (std::string _input) {
        std::string s;
        if (!private_node_handle_->hasParam(_input)) {
            ROS_ERROR_STREAM("The string (" << _input << ") does not exist in the parameter server tree. ");
            return false;
        } else
            return true;

    }

    /// <summary>
    /// Run each heuristic into processing pipeline
    /// </summary>
    /// <returns>true if the heuristic run correctly, otherwise false </returns>
    bool DietEstimationSkill::runMethods () {


        for (size_t i = 0; i < estimationPipelineArrPtr_->size(); ++i) {

            estimationPipelineArrPtr_->at(i)->setInitialList(candidate_list_);
            if(!estimationPipelineArrPtr_->at(i)->run()){
                return false;
            }
            candidate_list_ = estimationPipelineArrPtr_->at(i)->getResultList();

        }

        feedback (75);

        return true;
    }

    /// <summary>
    /// Get the best candidate after the processing pipeline execution
    /// </summary>
    /// <returns>false if the candidate list is empty, otherwise true </returns>
    bool DietEstimationSkill::getResult(){

        if(candidate_list_.empty()){
            ROS_ERROR_STREAM("Candidate list is empty. Unable to select best candidate.");
            return false;
        }


        std::sort( candidate_list_.begin(),
                   candidate_list_.end(),
                   tuple_operations::sortByThirdDescending<DietEstimationBase::CandidateTuple>);

        msg_result_candidate_ = std::get<0>(candidate_list_.at(0));

        ROS_INFO_STREAM("Selected candidate: " << msg_result_candidate_);
        feedback (100);
        return true;

    }

}
