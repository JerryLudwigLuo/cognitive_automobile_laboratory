#include "path_generator.hpp"

int main(int argc, char* argv[]) {

    ros::init(argc, argv, "path_generator_node");

    path_generator_ros_tool::PathGenerator path_generator(ros::NodeHandle(), ros::NodeHandle("~"));

    ros::shutdown();

    return EXIT_SUCCESS;
}
