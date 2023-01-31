#include "headers/data_buff.h"

DataBuff::DataBuff()
{
    ClearBuff();
}

void DataBuff::CheckBuffLength()
{
    while(lead_imu_data_buff_.size() > 100)
    {
        lead_imu_data_buff_.pop_front();
    }

    while(follow_imu_data_buff_.size() > 100)
    {
        follow_imu_data_buff_.pop_front();
    }

    while(lead_wheel_rot_rate_data_buff_.size() > 100)
    {
        lead_wheel_rot_rate_data_buff_.pop_front();
    }

    while(follow_wheel_rot_rate_data_buff_.size() > 100)
    {
        follow_wheel_rot_rate_data_buff_.pop_front();
    }

    while(lead_wheel_steer_angle_data_buff_.size() > 100)
    {
        lead_wheel_steer_angle_data_buff_.pop_front();
    }

    while(follow_wheel_steer_angle_data_buff_.size() > 100)
    {
        follow_wheel_steer_angle_data_buff_.pop_front();
    }

    while(scan_point_data_buff_.size() > 10)
    {
        scan_point_data_buff_.pop_front();
    }

    while(camera_rel_pose_data_buff_.size() > 10)
    {
        camera_rel_pose_data_buff_.pop_front();
    }

}

void DataBuff::ClearBuff()
{
    lead_imu_data_buff_.clear();
    follow_imu_data_buff_.clear();

    lead_wheel_rot_rate_data_buff_.clear();
    follow_wheel_rot_rate_data_buff_.clear();

    lead_wheel_steer_angle_data_buff_.clear();
    follow_wheel_steer_angle_data_buff_.clear();

    scan_point_data_buff_.clear();
    camera_rel_pose_data_buff_.clear();
}