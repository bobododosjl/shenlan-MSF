//
// Created by sjl on 21-1-27.
//

#include "lidar_localization/models/registration/icp_manual_registration.hpp"
#include "glog/logging.h"
#include <Eigen/Dense>

namespace lidar_localization {

    ICPManualRegistration::ICPManualRegistration(const YAML::Node& node)
            :  kdtree_ptr_(new pcl::KdTreeFLANN<CloudData::POINT>)  {
        float max_correspond_dis  =  node["max_correspondence_distance"].as<float>(); ///获取参数
        int  max_iter  =  node["max_iter"].as<int>();      ///最大迭代次数
        SetRegistrationParam(max_correspond_dis,  max_iter);
    }

    ICPManualRegistration::ICPManualRegistration(float max_correspond_dis,
                                                   int max_iter)
            :  kdtree_ptr_(new pcl :: KdTreeFLANN<CloudData :: POINT> )   {
        SetRegistrationParam(max_correspond_dis,max_iter);
    }

    bool  ICPManualRegistration::SetRegistrationParam(float max_correspond_dis, int  max_iter){
        max_correspond_distance_  = max_correspond_dis;
        max_iterator_ =  max_iter;
        LOG(INFO)     <<  "ICP_Manual的匹配参数为 :   "   << std::endl
                      << "max_correspond_dis:  "       <<  max_correspond_dis  << ", "
                      << "max_iter:  "      <<  max_iter   <<  std::endl
                      << std :: endl;
        return true;
    }

    bool ICPManualRegistration::SetInputTarget(
            const CloudData::CLOUD_PTR& input_target)   {
        target_cloud_.reset(new CloudData::CLOUD);
        target_cloud_ = input_target;
        kdtree_ptr_ ->setInputCloud(input_target);
        return   true;
    }

    bool ICPManualRegistration::ScanMatch(const CloudData::CLOUD_PTR   &input_source,
                                          const Eigen::Matrix4f& predict_pose,
                                          CloudData::CLOUD_PTR& result_cloud_ptr,
                                          Eigen::Matrix4f& result_pose) {
        transformation_  = predict_pose;
        rotation_matrix_ = transformation_.block<3,  3>(0,  0) ;    ///取旋转矩阵
        translation_  = transformation_.block<3,  1>(0,  3);                      ///取平移矩阵

        calculateTrans(input_source);       ///计算变换矩阵


        pcl::transformPointCloud(*input_source,   *result_cloud_ptr,  transformation_);   /// 对点云进行变换
        result_pose = transformation_;

        return true;
    }

    void ICPManualRegistration::calculateTrans(const CloudData::CLOUD_PTR   &input_source){
        CloudData::CLOUD_PTR  transformed_cloud(new CloudData::CLOUD);
        int knn = 1;     /// 进行 1nn的搜索
        int iterator_num = 0;
        while(iterator_num < max_iterator_)
        {
            pcl::transformPointCloud(*input_source,*transformed_cloud,transformation_);    /// 对点云进行变换
            Eigen::Matrix<float,6,6> Hessian;
            Eigen::Matrix<float,6,1>B;
            Hessian.setZero();
            B.setZero();     /// 归零

            for(size_t i =0; i < transformed_cloud->size();  ++i)
            {
                auto ori_point = input_source->at(i);
                if(!pcl::isFinite(ori_point))
                    continue;
                auto transformed_point = transformed_cloud->at(i);
                std::vector<float> distances;
                std::vector<int>indexs;
                kdtree_ptr_->nearestKSearch(transformed_point,knn,indexs,distances);      /// knn搜索
                if(distances[0] > max_correspond_distance_)
                {
                    continue;
                }
                Eigen::Vector3f closet_point  = Eigen::Vector3f(target_cloud_->at(indexs[0]).x,   target_cloud_->at(indexs[0]).y ,
                                                                target_cloud_->at(indexs[0]).z );
                /// 计算 原始点 与  最邻近点 的 距离
                Eigen::Vector3f err_dis =
                        Eigen::Vector3f(transformed_point.x,transformed_point.y,transformed_point.z) - closet_point;

                Eigen::Matrix<float,3,6> Jacobian(Eigen::Matrix<float,3,6>::Zero());
                Jacobian.leftCols<3>() = Eigen::Matrix3f::Identity();
                Jacobian.rightCols<3>() =
                        -rotation_matrix_* Sophus::SO3f::hat(Eigen::Vector3f(ori_point.x,ori_point.y,ori_point.z)) ;
                Hessian  +=  Jacobian.transpose()* Jacobian;
                B += -Jacobian.transpose()*err_dis;
            }
            iterator_num++;
            if(Hessian.determinant() == 0)
            {
                continue;
            }
            Eigen::Matrix<float,6,1> delta_x =  Hessian.inverse()*B;

            translation_ += delta_x.head<3>();
            auto  delta_rotation = Sophus::SO3f::exp(delta_x.tail<3>());
            rotation_matrix_ *= delta_rotation.matrix();

            transformation_.block<3,3>(0,0) = rotation_matrix_;
            transformation_.block<3,1>(0,3) = translation_;

        }

    }
}