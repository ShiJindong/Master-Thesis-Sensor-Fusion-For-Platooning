#include "headers/data_type.h"

void IMUData::SynchIMU(const IMUData &front_imu_data, 
                       const IMUData &back_imu_data,  
                       const double synch_time,
                       IMUData &synch_imu_data)
{
    double front_scale = (back_imu_data.timestamp_ - synch_time) / (back_imu_data.timestamp_ - front_imu_data.timestamp_);
    double back_scale = (synch_time - front_imu_data.timestamp_) / (back_imu_data.timestamp_ - front_imu_data.timestamp_);
    synch_imu_data.timestamp_ = synch_time;
    // Use linear Interpolation
    synch_imu_data.linear_acceleration_.x = front_imu_data.linear_acceleration_.x * front_scale + back_imu_data.linear_acceleration_.x * back_scale;
    synch_imu_data.linear_acceleration_.y = front_imu_data.linear_acceleration_.y * front_scale + back_imu_data.linear_acceleration_.y * back_scale;
    synch_imu_data.linear_acceleration_.z = front_imu_data.linear_acceleration_.z * front_scale + back_imu_data.linear_acceleration_.z * back_scale;
    synch_imu_data.angular_velocity_.x = front_imu_data.angular_velocity_.x * front_scale + back_imu_data.angular_velocity_.x * back_scale;
    synch_imu_data.angular_velocity_.y = front_imu_data.angular_velocity_.y * front_scale + back_imu_data.angular_velocity_.y * back_scale;
    synch_imu_data.angular_velocity_.z = front_imu_data.angular_velocity_.z * front_scale + back_imu_data.angular_velocity_.z * back_scale;
}

