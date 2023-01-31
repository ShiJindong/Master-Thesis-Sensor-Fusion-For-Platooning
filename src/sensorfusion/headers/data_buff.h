#pragma once

#include "data_type.h"
#include <deque>

class DataBuff
{
public:
    std::deque<IMUData> lead_imu_data_buff_;
    std::deque<IMUData> follow_imu_data_buff_;

    std::deque<WheelRotRateData> lead_wheel_rot_rate_data_buff_;
    std::deque<WheelRotRateData> follow_wheel_rot_rate_data_buff_;

    std::deque<WheelSteerAngleData> lead_wheel_steer_angle_data_buff_;
    std::deque<WheelSteerAngleData> follow_wheel_steer_angle_data_buff_;

    std::deque<ScanPointData> scan_point_data_buff_;
    std::deque<CameraRelPose> camera_rel_pose_data_buff_;

    DataBuff();
    ~DataBuff() = default;

    void CheckBuffLength();
    void ClearBuff();
};