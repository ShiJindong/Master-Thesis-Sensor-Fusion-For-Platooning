#pragma once

#include <iostream>
#include <deque>
#include "Eigen/Dense"
#include "yaml.h"
#include "data_type.h"

struct WheelParams
{
    double wheel_radius_front, wheel_radius_rear;         // Radius of front tires and rear tires
    double half_track_front, half_track_rear;             // Half track width of front axle and rear axle
    double lf, lr;                                        // Absolute distance between front axle and gravity center; Absolute distance between rear axle and gravity center

    int step_size;                                         // Step size of collected past wheel data
    double wheel_rot_rate_change_threshold;
    double velocity_deviation_threshold;
};

class WheelSlipDetector
{
private:
    WheelParams wheel_params_;
    Eigen::Vector4d half_track_;                            // half_track_[i = 0 ~ 3 for FL, FR, RL, RR-Tire]
    Eigen::Vector4d distance_axle_to_CG_;                   // distance_axle_to_CG_[i = 0 ~ 3 for FL, FR, RL, RR-Tire]
    Eigen::Vector4d wheel_radius_;                          // wheel_radius_[i = 0 ~ 3 for FL, FR, RL, RR-Tire]

    std::deque<WheelData> wheel_data_his_;                  // Collected historical (FL, FR, RL, RR-) wheels data
    WheelData wheel_data_present_;                          // (FL, FR, RL, RR-) wheels data at present 
    
    double ax_, wz_;                                        // Longitudinal acceleration and yaw rate from IMU
    double beta_;                                           // Sideslip angle [rad] estimated by sideslip_angle_estimator
    double vx_pred_;                                        // Pre-Estimation of velocity [m/s] from velocity_estimator
    Eigen::Vector4d wheel_speed_pred_;                      // Pre-Estimation of (FL, FR, RL, RR-) wheel speed [m/s]

    int valid_wheel_count_;                                 // If n wheels have valid data, then valid_wheel_count_ = n

public:
    bool is_initialized_;
    Eigen::Vector4d wheel_speed_CG_;                        // (FL, FR, RL, RR-) Wheel Speed [m/s] in Center of Gravity
    Eigen::Vector4i is_wheel_data_valid_;                   // Four flags for four wheels. If the wheel data (i = 0 ~ 3 for FL, FR, RL, RR) is valid (does not have wheel slip), is_wheel_data_valid_[i] is equal 1.
    Eigen::Vector4d wheel_rot_rate_change_;
    Eigen::Vector4d velocity_deviation_;
    double vx_best_;
    
public:
    WheelSlipDetector();
    ~WheelSlipDetector() = default;
    bool Run(const WheelData &wheel_data_present, 
             const double ax, const double wz, const double beta, 
             const double vx_pred, double &vx_best);
    void ReadConfig();
    void TransformWheelSpeed();
    void DetectWheelSlip();
    double SelectBestWheelSpeed();
};