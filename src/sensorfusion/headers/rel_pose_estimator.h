#pragma once

#include <iostream>
#include <vector>

#include "Eigen/Dense"
#include "yaml.h"

#include "data_buff.h"
#include "tools.h"

struct RelPoseParameters
{
    // Process noise variance
    double ax_process_noise_var, wz_dot_process_noise_var, beta_dot_process_noise_var;

    // Measurement noise variance
    double lidar_dx_noise_var, lidar_dy_noise_var, lidar_dphi_noise_var;
    double camera_dx_noise_var, camera_dy_noise_var, camera_dphi_noise_var;
    double vx_estimated_noise_var, wz_IMU_noise_var, ax_IMU_noise_var, beta_estimated_noise_var;

    // Initial value of states variance
    Eigen::MatrixXd P0;
};

class RelPoseEstimator{
private:
    // Parameters of rel_pose_estimator
    RelPoseParameters rel_params_;
    // Time interval of computing
    double delta_t_;

    // Dimension of state
    int dim_X_;
    // Dimension of process noise, we have process noise Nu = [nu_ax, nu_wz_dot, nu_beta_dot]
    int dim_Nu_;
    // Dimension of lidar measurement
    int dim_Z_lidar_;
    // Dimension of camera measurement
    int dim_Z_camera_;
    // Dimension of lead vehicle measurement
    int dim_Z_lead_;
    // Dimension of augmented state  (L_ = dim_X_ + dim_Nu_)
    int L_;


    /* 
     * State vector:  
     * X_ = [longitudinal distance, lateral distance, yaw difference, longitudinal velocity of lead vihicle, yaw rate of lead vehicle, longitudinal acceleration of lead vehicle, sideslip angle of lead vehicle]
     *    = [dx, dy, dphi, vx2, wz2, ax2, beta2] 
     * Note: [dx, dy, dphi] is relative pose between lead vehicle rear and follow vehicle front
     */           
    Eigen::VectorXd X_; 
    // Augmented state vector: X_aug_ = [X_, Nu] = [dx, dy, dphi, vx2, wz2, ax2, beta2, nu_ax, nu_wz_dot, nu_beta_dot]
    Eigen::VectorXd X_aug_;
    // state covariance matrix
    Eigen::MatrixXd P_;
    // augmented state covariance matrix
    Eigen::MatrixXd P_aug_;
    // Sigma points matrix with dimension: (dim_X_, 2 * L_ + 1)
    Eigen::MatrixXd Sigma_Points_;
    // Augmented sigma points matrix with dimension: (L_, 2 * L_ + 1)
    Eigen::MatrixXd Sigma_Points_aug_;
    // Covariance matrix of process noise
    Eigen::MatrixXd process_noise_cov_;
    // Covariance matrix of lidar measurement noise 
    Eigen::MatrixXd lidar_noise_cov_;
    // Covariance matrix of camera measurement noise 
    Eigen::MatrixXd camera_noise_cov_;
    // Covariance matrix of lead vehicle
    Eigen::MatrixXd lead_noise_cov_;
    
    
    /*
     * Input:
     * u = [longitudinal velocity of follow vehicle, yaw rate of follow vehicle, longitudinal acceleration of follow vehicle, sideslip angle of follow vehicle]
     *   = [vx1, wz1, ax1, beta1]      
     */
    double vx1_, wz1_, ax1_, beta1_;


    // Lidar Measurement: Z_lidar_ = [dx_lidar, dy_lidar, dphi_lidar] 
    Eigen::Vector3d Z_lidar_;
    // Camera Measurement: Z_camera_ = [dx_camera, dy_camera, dphi_camera] 
    Eigen::Vector3d Z_camera_;
    /*
     * Lead Vehicle Measurement:  
     * Z_lead_ = [longitudinal velocity of lead vehicle, yaw rate of lead vehicle, longitudinal acceleration of lead vehicle, sideslip angle of lead vehicle]
     *         = [vx2, wz2, ax2, beta2]
     */
    Eigen::Vector4d Z_lead_;


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

    // Absolute Pose of follow vehicle (w.r.t. Center of Gravity)
    Eigen::Vector3d abs_pose_follow_CG_;
    // Absolute Pose of follow vehicle (w.r.t. vehicle rear (Origin Fr1))
    Eigen::Vector3d abs_pose_follow_R_;

    // Timestamp of last observation
    double last_obs_time_;

public:
    // flag for initialization
    bool is_initialized_; 

    // Timestamp of Filter [s]
    double time_;
    
    RelPoseEstimator();
    ~RelPoseEstimator() = default;

    void Run(std::deque<IMUVelocitySideslipData> &follow_data_buff,
             std::deque<IMUVelocitySideslipData> &lead_data_buff,
             std::deque<LidarRelPose> &lidar_rel_pose_data_buff,
             std::deque<CameraRelPose> &camera_rel_pose_data_buff);    

    void Init(std::deque<LidarRelPose> &lidar_rel_pose_data_buff,
              std::deque<CameraRelPose> &camera_rel_pose_data_buff);    

    void ReadConfig();
    void GenerateSigmaPoints();
    void Predict();
    void UpdateLidar();  
    void UpdateCamera(int num_detected_aruco_markers);
    void UpdateLeadStates();  
    void Update(const Eigen::VectorXd &Z, const Eigen::MatrixXd &Sigma_Points_Z, const Eigen::MatrixXd &measurement_noise_cov); 
    Eigen::Vector3d GetRelPose();
    Eigen::Vector4d GetLeadStates();

    void ComputeAbsPose();
    Eigen::Vector3d GetAbsPoseFollow();

    void GetStatesStandardDeviation(Eigen::VectorXd &states_stdd);      
};