#pragma once

#include "data_type.h"
#include "data_buff.h"
#include "LidarDetectionAndTracking.h"
#include "rel_pose_estimator.h"
#include "velocity_estimator.h"
#include "sideslip_angle_estimator.h"
#include "tools.h"
#include "Eigen/Dense"
#include <iostream>
#include <deque>

#define ShowTimeLine 1

class VehiclePositioningFlow
{
private:
    RelPoseEstimator rel_pose_estimator_;

    VelocityEstimator follow_velocity_estimator_;
    SideslipAngleEstimator follow_sideslip_angle_estimator_;
    
    VelocityEstimator lead_velocity_estimator_;
    SideslipAngleEstimator lead_sideslip_angle_estimator_;

    // Lidar Tracker
    const int LidarL_ = 0;
    const int LidarR_ = 1;
    LidarTracker lidar_tracker_;

    // Synchronised IMU, Velocity, Sideslip Angle Data into Data Buffer
    std::deque<IMUVelocitySideslipData> follow_data_buff_;
    std::deque<IMUVelocitySideslipData> lead_data_buff_;
    std::deque<LidarRelPose> lidar_rel_pose_data_buff_;
    std::deque<CameraRelPose> camera_rel_pose_data_buff_;

    // Timestamp of last camera image data
    double last_camera_timestamp_;
    
    // Timestamp of last valid lidar relative pose
    double last_valid_lidar_data_timestamp_;
    
public:
    bool is_initialized_; 
    double estimated_follow_longi_velocity_, estimated_lead_longi_velocity_;
    double estimated_follow_sideslip_angle_, estimated_lead_sideslip_angle_;

    // Relative pose between leadbus rear and followbus front [dx, dy, dphi]
    Eigen::Vector3d lidar_rel_pose_;                 
    Eigen::Vector3d camera_rel_pose_; 
    Eigen::Vector3d fusion_rel_pose_;

    // Absolute pose of follow vehicle (w.r.t the vehicle rear (origin of Fr1))
    Eigen::Vector3d estimated_follow_abs_pose_;      

    // [vx2, wz2, ax2, beta2]
    Eigen::Vector4d estimated_lead_states_;

    int num_detected_markers_;
    int is_lidar_rel_pose_valid_;
    int is_camera_rel_pose_valid_;

    // Standard Deviation of estimated states in rel_pose_estimator
    Eigen::VectorXd states_stdd_;

    // Only for plotten in IPGControl
    Eigen::Vector4d follow_wheel_speed_CG_;
    Eigen::Vector4i is_follow_wheel_data_valid_;                   
    Eigen::Vector4d follow_wheel_rot_rate_change_;
    Eigen::Vector4d follow_velocity_deviation_;

public:
    VehiclePositioningFlow();
    ~VehiclePositioningFlow() = default;
    void Init();
    void Run(std::shared_ptr<DataBuff> data_buff);
    void ReadConfig();
};
