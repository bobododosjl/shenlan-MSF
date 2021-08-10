//
// Created by sjl on 21-1-27.
//

#ifndef LIDAR_LOCALIZATION_ICP_MANUAL_REGISTRATION_HPP
#define LIDAR_LOCALIZATION_ICP_MANUAL_REGISTRATION_HPP

#include <pcl/registration/icp.h>
#include <pcl/kdtree/kdtree_flann.h>
#include "lidar_localization/models/registration/registration_interface.hpp"         //配准接口
#include "/home/bobododo/GNC/MSF/chapter2/catkin_ws/src/lidar_localization/modules/sophus/se3.hpp"                // 添加 se3

namespace lidar_localization {
    class ICPManualRegistration: public RegistrationInterface {
    public:
        ICPManualRegistration(const YAML::Node  &node);
        ICPManualRegistration(float max_correspond_dis,  int max_iter);

        bool SetInputTarget(const CloudData::CLOUD_PTR   &input_target) override;
        bool ScanMatch(const CloudData::CLOUD_PTR   &input_source,
                       const Eigen::Matrix4f    &predict_pose,
                       CloudData::CLOUD_PTR   &result_cloud_ptr,
                       Eigen::Matrix4f   &result_pose) override;

    private:
        bool SetRegistrationParam(float max_correspond_dis,int  max_iter);
        void calculateTrans(const CloudData::CLOUD_PTR   &input_cloud );       // 计算旋转矩阵

    private:
        CloudData::CLOUD_PTR target_cloud_;
        pcl::KdTreeFLANN<CloudData::POINT>::Ptr  kdtree_ptr_;
        float max_correspond_distance_;     // 阈值
        int max_iterator_;                                     //最大迭代次数

        Eigen::Matrix3f  rotation_matrix_;       //旋转矩阵
        Eigen::Vector3f  translation_;                 //平移矩阵
        Eigen::Matrix4f  transformation_;       // 转换矩阵

    };
}
#endif //LIDAR_LOCALIZATION_ICP_MANUAL_REGISTRATION_HPP
