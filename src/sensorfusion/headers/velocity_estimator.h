#pragma once

#include "Eigen/Dense"
#include "yaml.h"

#include "kalman_filter.h"
#include "data_buff.h"
#include "wheel_slip_detector.h"
#include "data_type.h"

class VelocityEstimator
{
private:
    const double g_ = 9.81;              // Gravitational Acceleration
    double delta_t_;
    double time_;

    double ax_, wz_;                    // Longitudinal acceleration and yaw rate from IMU
    double beta_;                       // Sideslip angle from sideslip_angle_estimator

    KalmanFilter velocity_kf_; 

    Eigen::Vector2d X_prior_;
    Eigen::Matrix2d P0_;
    Eigen::Matrix2d Q_;
    Eigen::Matrix<double, 1, 1> R_;
    Eigen::Matrix2d A_;
    Eigen::Matrix<double, 1, 2> H_;

    std::deque<IMUData> imu_data_buff_;
    std::deque<WheelData> wheel_data_buff_;

    IMUData current_imu_data_;
    IMUData last_imu_data_;
    WheelData current_wheel_data_;
    WheelSteerAngleData synced_wheel_steer_angle_data_;
    double sideslip_angle_estimated_;

public:
    bool is_initialized_; 
    double vx_best_;
    
    WheelSlipDetector wheel_slip_detector_;
    bool is_at_least_one_wheel_data_valid_;
    
    Eigen::Vector4d wheel_speed_CG_;                 
    Eigen::Vector4i is_wheel_data_valid_;                   
    Eigen::Vector4d wheel_rot_rate_change_;
    Eigen::Vector4d velocity_deviation_;

    double init_velocity_;
    

public:
    VelocityEstimator();
    void Run(std::deque<IMUData> &imu_data_buff, 
             std::deque<WheelRotRateData> &wheel_rot_rate_data_buff,   
             std::deque<WheelSteerAngleData> &wheel_steer_angle_data_buff,  
             double sideslip_angle_estimated,
             std::deque<IMUVelocitySideslipData> &imu_velocity_sideslip_data_buff);

    void ReadConfig();
    void Init();
    double GetLongiVelocity();
    double GetSlope();
    void SetInitVelocity(double init_velocity);

    void SubscribeIMUData(std::deque<IMUData> &origin_imu_data_buff, 
                          std::deque<IMUData> &subscribed_imu_data_buff);

    void SubscribeWheelData(std::deque<WheelRotRateData> &origin_wheel_rot_rate_data_buff, 
                            std::deque<WheelSteerAngleData> &origin_wheel_steer_angle_data_buff, 
                            std::deque<WheelData> &subscribed_wheel_data_buff);
   
    bool HasIMUData();
    bool HasWheelData();

    void Predict(IMUData &imu_data);
    void Update(IMUData &imu_data, WheelData &wheel_data);

};
