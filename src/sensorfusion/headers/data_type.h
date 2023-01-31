#pragma once

#include <deque>
#include "Eigen/Dense"
#include "Vehicle/Sensor_LidarRSI.h"

struct RelPose 
{
    RelPose() = default;
    ~RelPose() = default;
    double dx = 0.0;
    double dy = 0.0;
    double dphi = 0.0;
};

struct LinearAcceleration 
{
    LinearAcceleration() = default;
    ~LinearAcceleration() = default;
    double x = 0.0;
    double y = 0.0;
    double z = 0.0;
};

struct AngularVelocity 
{
    AngularVelocity() = default;
    ~AngularVelocity() = default;
    double x = 0.0;
    double y = 0.0;
    double z = 0.0;
};

class IMUData
{
public:
    IMUData(LinearAcceleration linear_acceleration, AngularVelocity angular_velocity, double timestamp):
            linear_acceleration_(linear_acceleration), angular_velocity_(angular_velocity), timestamp_(timestamp) {}
    IMUData() = default;
    ~IMUData() = default;

    static void SynchIMU(const IMUData &front_imu_data, 
                         const IMUData &back_imu_data,  
                         const double synch_time,
                         IMUData &synch_imu_data);

    LinearAcceleration linear_acceleration_;
    AngularVelocity angular_velocity_;
    /* In CarMaker simulation, we use simulation time (double value) after simulation starts as timestamp */
    double timestamp_ = 0.0;              // in second           
};

class WheelRotRateData
{
public:
    WheelRotRateData(Eigen::Vector4d wheel_rot_rate, double timestamp):
                     wheel_rot_rate_(wheel_rot_rate), timestamp_(timestamp) {}
    WheelRotRateData() = default;
    ~WheelRotRateData() = default;

    Eigen::Vector4d wheel_rot_rate_;                // wheel_rot_rate_[i = 0 ~ 3]: Wheel Rotation Rate [rad/s] of (FL, FR, RL, RR)-Tires
    double timestamp_ = 0.0;                  
};

class WheelSteerAngleData
{
public:
    WheelSteerAngleData(Eigen::Vector4d wheel_steer_angle, double timestamp):
                        wheel_steer_angle_(wheel_steer_angle), timestamp_(timestamp) {}
    WheelSteerAngleData() = default;
    ~WheelSteerAngleData() = default;

    Eigen::Vector4d wheel_steer_angle_;             // wheel_steer_angle_[i = 0 ~ 3]: Wheel Steering Angle [rad] of (FL, FR, RL, RR)-Tires
    double timestamp_ = 0.0;                  
};


/* Synchronised Wheel Rotation Rate and Wheel Steering Angle Data (use Wheel Rotation Rate Data Timestamp) */
class WheelData
{
public:
    WheelData(Eigen::Vector4d wheel_rot_rate, Eigen::Vector4d wheel_steer_angle, double timestamp):
                    wheel_rot_rate_(wheel_rot_rate), wheel_steer_angle_(wheel_steer_angle), timestamp_(timestamp) {}
    WheelData() = default;
    ~WheelData() = default;

    Eigen::Vector4d wheel_rot_rate_;                // Wheel Rotation Rate [rad/s] of (FL, FR, RL, RR)-Tires
    Eigen::Vector4d wheel_steer_angle_;             // Wheel Steering Angle [rad] of (FL, FR, RL, RR)-Tires
    double timestamp_ = 0.0;                  
};


/* Sychronised IMU, Velocity, Sideslip Angle Data (use IMU Data Timestamp) */
class IMUVelocitySideslipData
{
public:
    IMUVelocitySideslipData(LinearAcceleration linear_acceleration, AngularVelocity angular_velocity, 
                            double vx, double vy, double sideslip_angle, double timestamp) :
                            linear_acceleration_(linear_acceleration), angular_velocity_(angular_velocity), 
                            vx_(vx), vy_(vy), sideslip_angle_(sideslip_angle), timestamp_(timestamp) {}
    IMUVelocitySideslipData() = default;
    ~IMUVelocitySideslipData() = default;

    LinearAcceleration linear_acceleration_;
    AngularVelocity angular_velocity_;
    double vx_ = 0.0;                       
    double vy_ = 0.0; 
    double sideslip_angle_ = 0.0;
    double timestamp_ = 0.0; 

};


class ScanPointData
{
public:
    ScanPointData() = default;
    ~ScanPointData() = default;

    tScanPoint* scan_point_ptr_[2];         // scan_point_ptr_[0]: scanpoints of left lidar, scan_point_ptr_[1]: scanpoints of right lidar, 
    int num_ScanPoints_[2];                 // num_ScanPoints_[0]: scanpoints number of left lidar, num_ScanPoints_[1]: scanpoints number of right lidar, 
    
    double timestamp_ = 0.0;                // Can we expect that two lidar at the same time triggered?
};


class LidarRelPose
{
public:
    LidarRelPose(RelPose rel_pose, double timestamp):
                  rel_pose_(rel_pose), timestamp_(timestamp) {}
    LidarRelPose() = default;
    ~LidarRelPose() = default;

    RelPose rel_pose_;                      // Calculated Relative Pose from Lidar Tracker
    double timestamp_ = 0.0;

};

class CameraRelPose
{
public:
    CameraRelPose(RelPose rel_pose, double timestamp):
                  rel_pose_(rel_pose), timestamp_(timestamp) {}
    CameraRelPose() = default;
    ~CameraRelPose() = default;

    RelPose rel_pose_;                      // Calculated Relative Pose from Camera Tracker
    int num_detected_aruco_markers = 0;
    double timestamp_ = 0.0;
};


