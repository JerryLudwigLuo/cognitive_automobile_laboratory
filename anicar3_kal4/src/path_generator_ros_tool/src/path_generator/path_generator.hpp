#pragma once

#include <dynamic_reconfigure/server.h>
#include <ros/forwards.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <Eigen/Eigen>
#include <Eigen/StdVector>

#include "path_generator_ros_tool/PathGeneratorInterface.h"
EIGEN_DEFINE_STL_VECTOR_SPECIALIZATION(Eigen::Affine3d);

namespace path_generator_ros_tool {

class PathGenerator {

    using Interface = PathGeneratorInterface;

    using Msg = std_msgs::Header;

public:
    PathGenerator(ros::NodeHandle, ros::NodeHandle);

private:
    void reconfigureRequest(const Interface::Config&, uint32_t);
    void homogenizePoints();
    void toRoadMap(std::string);
    void mapFitting(std::string);

    Interface interface_;
    dynamic_reconfigure::Server<Interface::Config> reconfigureServer_;
    std::vector<Eigen::Affine3d> trajectory_;
    tf2_ros::Buffer tfBuffer_;
    tf2_ros::TransformListener tfListener_{tfBuffer_};
    tf2_ros::TransformBroadcaster tfBroadcaster_;

};
} // namespace path_generator_ros_tool
