#include <diet_estimation_skill_server/diet_estimation_skill_server.h>
#include <diet_estimation_skill_server/common/verbosity_levels.h>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "diet_estimation_skill");

    ros::NodeHandlePtr node_handle(new ros::NodeHandle());
    ros::NodeHandlePtr private_node_handle(new ros::NodeHandle("~"));

     std::string ros_verbosity_level;
     private_node_handle->param("ros_verbosity_level", ros_verbosity_level, std::string("DEBUG"));
     diet_estimation_skill::verbosity_levels::setVerbosityLevelROS(ros_verbosity_level);

    diet_estimation_skill::DietEstimationSkill skill;
    skill.setupSkillConfigurationFromParameterServer(node_handle, private_node_handle);
    skill.start();

    ros::spin();

}
