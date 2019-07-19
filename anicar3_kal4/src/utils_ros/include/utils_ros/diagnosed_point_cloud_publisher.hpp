#pragma once

#include <diagnostic_updater/publisher.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/point_cloud.h>

namespace utils_ros {

template <typename T>
class DiagnosedPointCloudPublisher : public diagnostic_updater::TopicDiagnostic {
public:
    DiagnosedPointCloudPublisher(const ros::Publisher& pub,
                                 diagnostic_updater::Updater& diag,
                                 const diagnostic_updater::FrequencyStatusParam& freq,
                                 const diagnostic_updater::TimeStampStatusParam& stamp)
            : TopicDiagnostic(pub.getTopic(), diag, freq, stamp), publisher_(pub) {
    }

    ~DiagnosedPointCloudPublisher() = default;

    void publish(const typename pcl::PointCloud<T>::ConstPtr& msg) {
        tick(pcl_conversions::fromPCL(msg->header.stamp));
        publisher_.publish(*msg);
    }

    const ros::Publisher& getPublisher() const {
        return publisher_;
    }

private:
    ros::Publisher publisher_;
};
} // namespace utils_ros
