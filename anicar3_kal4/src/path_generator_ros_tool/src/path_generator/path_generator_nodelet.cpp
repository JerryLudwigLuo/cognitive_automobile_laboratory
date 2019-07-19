#include <memory>
#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>

#include "path_generator.hpp"

namespace path_generator_ros_tool {

class PathGeneratorNodelet : public nodelet::Nodelet {

    inline void onInit() override {
        impl_ = std::make_unique<PathGenerator>(getNodeHandle(), getPrivateNodeHandle());
    }
    std::unique_ptr<PathGenerator> impl_;
};
} // namespace path_generator_ros_tool

PLUGINLIB_EXPORT_CLASS(path_generator_ros_tool::PathGeneratorNodelet, nodelet::Nodelet);
