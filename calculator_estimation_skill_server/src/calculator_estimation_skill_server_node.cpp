#include <calculator_estimation_skill_server/calculator_estimation_skill_server.h>
#include <grasp_estimation_skill_server/common/verbosity_levels.h>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "calculator_estimation_skill");

    ros::NodeHandlePtr node_handle(new ros::NodeHandle());
    ros::NodeHandlePtr private_node_handle(new ros::NodeHandle("~"));

    std::string ros_verbosity_level;
    private_node_handle->param("ros_verbosity_level", ros_verbosity_level, std::string("DEBUG"));
    calculator_estimation_skill::verbosity_levels::setVerbosityLevelROS(ros_verbosity_level);

    calculator_estimation_skill::CalculatorEstimationSkill skill;
    skill.setupSkillConfigurationFromParameterServer(node_handle, private_node_handle);
    skill.start();


    ros::spin();

    skill.generateLog();

}
