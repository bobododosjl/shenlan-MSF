#ifndef LIDAR_LOCALIZATION_MODELS_SCAN_ADJUST_DISTORTION_ADJUST_CUSTOM_HPP_
#define LIDAR_LOCALIZATION_MODELS_SCAN_ADJUST_DISTORTION_ADJUST_CUSTOM_HPP_

#include <pcl/common/transforms.h>
#include <Eigen/Dense>

#include "lidar_localization/sensor_data/velocity_data.hpp"
#include "lidar_localization/sensor_data/cloud_data.hpp"

namespace lidar_localization {
class DistortionAdjustCustom {
    public:
        void SetMotionInfo(float scan_period, VelocityData velocity_data);
        bool AdjustCloud(CloudData::CLOUD_PTR& input_cloud_ptr, CloudData::CLOUD_PTR& output_cloud_ptr);

    private:
        inline Eigen::Matrix3f UpdateMatrix(float real_time);

    private:
        float scan_period_;
        Eigen::Vector3f velocity_;
        Eigen::Vector3f angular_rate_;

};
}   // namespace lidar_localization

#endif