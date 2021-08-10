#include "lidar_localization/models/scan_adjust/distortion_adjust_custom.hpp"

namespace lidar_localization {
void DistortionAdjustCustom::SetMotionInfo(float scan_period, VelocityData velocity_data) {
    scan_period_ = scan_period;
    velocity_ << velocity_data.linear_velocity.x, velocity_data.linear_velocity.y, velocity_data.linear_velocity.z;
    angular_rate_ << velocity_data.angular_velocity.x, velocity_data.angular_velocity.y, velocity_data.angular_velocity.z;
}

bool DistortionAdjustCustom::AdjustCloud(CloudData::CLOUD_PTR& input_cloud_ptr, CloudData::CLOUD_PTR& output_cloud_ptr) {
    CloudData::CLOUD_PTR origin_cloud_ptr(new CloudData::CLOUD(*input_cloud_ptr));
    output_cloud_ptr.reset(new CloudData::CLOUD());
    float start_orientation = atan2(origin_cloud_ptr->points[0].y, origin_cloud_ptr->points[0].x);

    Eigen::AngleAxisf t_V(start_orientation, Eigen::Vector3f::UnitZ());
    Eigen::Matrix3f rotate_matrix = t_V.matrix();
    Eigen::Matrix4f transform_matrix = Eigen::Matrix4f::Identity();
    transform_matrix.block<3,3>(0,0) = rotate_matrix.inverse();
    pcl::transformPointCloud(*origin_cloud_ptr, *origin_cloud_ptr, transform_matrix);

    velocity_ = rotate_matrix * velocity_;
    angular_rate_ = rotate_matrix * angular_rate_;

    int cloudSize = origin_cloud_ptr->points.size();
    float startOri = -atan2(origin_cloud_ptr->points[0].y, origin_cloud_ptr->points[0].x);
    float endOri = -atan2(origin_cloud_ptr->points[cloudSize - 1].y, origin_cloud_ptr->points[cloudSize - 1].x) + 2 * M_PI;

    if(endOri - startOri > 3 * M_PI){
        endOri -= 2 * M_PI;
    }else if(endOri - startOri < M_PI){
        endOri += 2* M_PI;
    }

    bool halfPassed = false;
    
    for(int i = 0; i < cloudSize; i++){

        float ori = -atan2(origin_cloud_ptr->points[i].y, origin_cloud_ptr->points[i].x);
        if(!halfPassed){
            if(ori < startOri - M_PI / 2){
                ori += 2 * M_PI;
            }else if(ori > startOri + M_PI * 3 / 2){
                ori -= 2 * M_PI;
            }

            if(ori - startOri > M_PI){
                halfPassed = true;
            }
        }else{
            ori += 2 * M_PI;
            if(ori < endOri - M_PI * 3 / 2){
                ori += 2 * M_PI;
            }else if(ori > endOri + M_PI / 2){
                ori -= 2 * M_PI;
            }
        }

        float real_time = (ori - startOri) / (endOri - startOri) * scan_period_;

        Eigen::Vector3f origin_point(origin_cloud_ptr->points[i].x,
                                    origin_cloud_ptr->points[i].y,
                                    origin_cloud_ptr->points[i].z);

        Eigen::Matrix3f current_matrix = UpdateMatrix(real_time);
        Eigen::Vector3f rotated_point = current_matrix * origin_point;
        Eigen::Vector3f adjusted_point = rotated_point + velocity_ * real_time;

        CloudData::POINT point;
        point.x = adjusted_point(0);
        point.y = adjusted_point(1);
        point.z = adjusted_point(2);
        output_cloud_ptr->points.push_back(point);
    }

    pcl::transformPointCloud(*output_cloud_ptr, *output_cloud_ptr, transform_matrix.inverse());
    return true;
}

Eigen::Matrix3f DistortionAdjustCustom::UpdateMatrix(float real_time){
    Eigen::Vector3f angle = angular_rate_ * real_time;
    Eigen::AngleAxisf t_Vz(angle(2), Eigen::Vector3f::UnitZ());
    Eigen::AngleAxisf t_Vy(angle(1), Eigen::Vector3f::UnitY());
    Eigen::AngleAxisf t_Vx(angle(0), Eigen::Vector3f::UnitX());
    Eigen::AngleAxisf t_V;
    t_V = t_Vz * t_Vy * t_Vx;
    return t_V.matrix();
}

}   // namespace lidar_localization