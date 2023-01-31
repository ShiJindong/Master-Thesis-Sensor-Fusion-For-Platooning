#pragma once

#include <iostream>
#include <vector>

#include "Eigen/Dense"
#include "yaml.h"

#include "data_buff.h"

#define DEBUG 0

struct SideslipParams
{
    // Vehicle weight
    double m;
    // Vehicle rotational inertia
    double Iz;
    // Cornering stiffnesses of the front and rear tires
    double Cf, Cr;
    // Absolute Distance from front axle and rear axle to vehicle center of gravity (CG)
    double lf, lr;
    // Absolute Value Threshold of longitudinal velocity 
    double vx_threshold;

    // Process noise variance
    double ax_process_noise_var, ay_process_noise_var, wz_dot_process_noise_var;
    // Measurement noise variance
    double ay_IMU_noise_var, vx_estimated_noise_var, wz_IMU_noise_var;

    // Initial State Covariance Matrix
    Eigen::MatrixXd P0;
};

class SideslipAngleEstimator
{
private:
    // Parameters of sideslip_angle_estimator
    SideslipParams sideslip_params_;
    // Time interval of computing
    double delta_t_;
    // Time when computing
    double time_;

    // Dimension of state
    int dim_X_;
    // Dimension of process noise, we have process noise Nu = [nu_ax, nu_ay, nu_wz_dot]
    int dim_Nu_;
    // Dimension of measurement
    int dim_Z_;
    // Dimension of augmented state  (L_ = dim_X_ + dim_Nu_)
    int L_;

    // State vector:  X_ = [longitudinal velocity, lateral velocity, yaw rate] = [vx, vy, wz]            
    Eigen::VectorXd X_; 
    // Augmented state vector: X_aug_ = [X_, nu_acc_x, nu_acc_y, nu_yaw_acc] = [vx, vy, wz, nu_ax, nu_ay, nu_wz_dot]
    Eigen::VectorXd X_aug_;
    // state covariance matrix
    Eigen::MatrixXd P_;
    // augmented state covariance matrix
    Eigen::MatrixXd P_aug_;
    // Measurement vector
    Eigen::VectorXd Z_;
    // Sigma points matrix with dimension: (dim_X_, 2 * L_ + 1)
    Eigen::MatrixXd Sigma_Points_;
    // Augmented sigma points matrix with dimension: (L_, 2 * L_ + 1)
    Eigen::MatrixXd Sigma_Points_aug_;
    // Covariance matrix of process noise
    Eigen::MatrixXd process_noise_cov_;
    // Covariance matrix of measurement noise 
    Eigen::MatrixXd measurement_noise_cov_;
    // Input u = [longitudinal acceleration from IMU, steer angle of front tires]
    double ax_IMU_, steer_angle_;
    // Measurement Z_ = [lateral acceleration from IMU, longitudinal velocity from velocity_estimator, yaw rate from IMU] = [ay_IMU, vx_estimated, wz_IMU]
    double ay_IMU_, vx_estimated_, wz_IMU_;
    // Estimated vehicle sideslip angle [rad]
    double sideslip_angle_;

    // Current IMU and Wheel Steering Angle Data from Data Buffer
    std::deque<IMUData> imu_data_buff_;
    IMUData current_imu_data_;
    WheelSteerAngleData current_wheel_steer_angle_data_;

    /* 
        Tuning paramters of UKF
            In paper "The Unscented Kalman Filter for Nonlinear Estimation" by Eric A. Wan and Rudolph van der Merwe, 
            the meanings of these tuning parameters and weights are introduced clearly.
    */
    // composite scaling parameter
    double lambda_;
    // alpha_ determines the spread of the sigma points around state mean and is usually set to a small positive value (0 < alpha < 1, e.g., 1e-3).
    double alpha_;
    // kappa_ is a secondary scaling parameter, which is usually set to 0
    double kappa_;
    // rho_ is used to incorporate prior knowledge of the states distribution (for Gaussian distributions, rho_ = 2 is optimal)
    double rho_;
    // weights for computing predicted state and predicted measurement
    std::vector<double> weights_m_;   
    // weights for computing covariance
    std::vector<double> weights_c_;   

public:
    // flag for initialization
    bool is_initialized_; 
    
    SideslipAngleEstimator();
    ~SideslipAngleEstimator() = default;

    void Run(std::deque<WheelSteerAngleData> &wheel_steer_angle_data_buff, 
             std::deque<IMUData> &imu_data_buff, 
             double vx_estimated);                 
    void Init();           
    void ReadConfig();
    bool ValidData();
    void GenerateSigmaPoints();
    void Predict();
    void Update();  
    void CalculateSideslipAngle();

    double GetLateralVelocity();            
    double GetSideslipAngle();       

    void SubscribeIMUData(std::deque<IMUData> &origin_imu_data_buff, 
                          std::deque<IMUData> &subscribed_imu_data_buff);      
    void SubscribeWheelSteerAngleData(std::deque<WheelSteerAngleData> &origin_wheel_steer_angle_data_buff);
    bool HasIMUData();
};