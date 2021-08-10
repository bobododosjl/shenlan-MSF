/*
 * @Description: 
 * @Author: Ren Qian
 * @Date: 2020-02-28 18:50:16
 */
#include "lidar_localization/sensor_data/pose_data.hpp"

namespace lidar_localization {

Eigen::Quaternionf PoseData::GetQuaternion() {
    Eigen::Quaternionf q;
    q = pose.block<3,3>(0,0);
    return q;
}

bool PoseData::SyncData(std::deque<PoseData>& UnsyncedData, std::deque<PoseData>& SyncedData, double sync_time) {
    // 传感器数据按时间序列排列，在传感器数据中为同步的时间点找到合适的时间位置
    // 即找到与同步时间相邻的左右两个数据
    // 需要注意的是，如果左右相邻数据有一个离同步时间差值比较大，则说明数据有丢失，时间离得太远不适合做差值
    while (UnsyncedData.size() >= 2) {
        // UnsyncedData.front().time should be <= sync_time:
        if (UnsyncedData.front().time > sync_time) 
            return false;
        // sync_time should be <= UnsyncedData.at(1).time:
        if (UnsyncedData.at(1).time < sync_time) {
            UnsyncedData.pop_front();
            continue;
        }

        // sync_time - UnsyncedData.front().time should be <= 0.2:
        if (sync_time - UnsyncedData.front().time > 2.0) {
            UnsyncedData.pop_front();
            return false;
        }
        // UnsyncedData.at(1).time - sync_time should be <= 0.2
        if (UnsyncedData.at(1).time - sync_time > 2.0) {
            UnsyncedData.pop_front();
            return false;
        }
        break;
    }
    if (UnsyncedData.size() < 2)
        return false;

    PoseData front_data = UnsyncedData.at(0);
    PoseData back_data = UnsyncedData.at(1);
    PoseData synced_data;

    double front_scale = (back_data.time - sync_time) / (back_data.time - front_data.time);
    double back_scale = (sync_time - front_data.time) / (back_data.time - front_data.time);
    synced_data.time = sync_time;
    synced_data.pose(0,3) = front_data.pose(0,3) * front_scale + back_data.pose(0,3) * back_scale;
    synced_data.pose(1,3) = front_data.pose(1,3) * front_scale + back_data.pose(1,3) * back_scale;
    synced_data.pose(2,3) = front_data.pose(2,3) * front_scale + back_data.pose(2,3) * back_scale;

    Eigen::Quaternionf synced_data_q(synced_data.pose.block<3,3>(0,0)); 
    Eigen::Quaternionf front_data_q(front_data.pose.block<3,3>(0,0)); 
    Eigen::Quaternionf back_data_q(front_data.pose.block<3,3>(0,0)); 
    // 四元数插值有线性插值和球面插值，球面插值更准确，但是两个四元数差别不大是，二者精度相当
    // 由于是对相邻两时刻姿态插值，姿态差比较小，所以可以用线性插值
    synced_data_q.x() = front_data_q.x() * front_scale + back_data_q.x() * back_scale;
    synced_data_q.y() = front_data_q.y() * front_scale + back_data_q.y() * back_scale;
    synced_data_q.z() = front_data_q.z() * front_scale + back_data_q.z() * back_scale;
    synced_data_q.w() = front_data_q.w() * front_scale + back_data_q.w() * back_scale;
    // 线性插值之后要归一化
    synced_data_q.normalize();

    synced_data.pose.block<3,3>(0,0) = synced_data_q.toRotationMatrix();

    SyncedData.push_back(synced_data);

    return true;
}

}