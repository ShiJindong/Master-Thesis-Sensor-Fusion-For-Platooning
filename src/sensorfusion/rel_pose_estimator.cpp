#include "headers/rel_pose_estimator.h"
#include "../User.h"

RelPoseEstimator::RelPoseEstimator()
{
    is_initialized_ = false;
    delta_t_ = 0;
    
    // Dimension
    dim_X_ = 7;
    dim_Nu_ = 3;
    dim_Z_lidar_ = 3;
    dim_Z_camera_ = 3;
    dim_Z_lead_ = 4;
    L_ = dim_X_ + dim_Nu_;

    // State, State Covariance, Sigma Points
    X_ = Eigen::VectorXd::Zero(dim_X_);
    X_aug_ = Eigen::VectorXd::Zero(L_);
    P_ = Eigen::MatrixXd::Zero(dim_X_, dim_X_);
    P_aug_ = Eigen::MatrixXd::Zero(L_, L_);
    Sigma_Points_ = Eigen::MatrixXd::Zero(dim_X_, 2 * L_ + 1);
    Sigma_Points_aug_ = Eigen::MatrixXd::Zero(L_, 2 * L_ + 1);

    // Noise Covariance
    process_noise_cov_ = Eigen::MatrixXd::Zero(dim_Nu_, dim_Nu_);
    lidar_noise_cov_ = Eigen::MatrixXd::Zero(dim_Z_lidar_, dim_Z_lidar_);
    camera_noise_cov_ = Eigen::MatrixXd::Zero(dim_Z_camera_, dim_Z_camera_);
    lead_noise_cov_ = Eigen::MatrixXd::Zero(dim_Z_lead_, dim_Z_lead_);

    // Input
    vx1_ = 0;
    wz1_ = 0;
    ax1_ = 0; 
    beta1_ = 0;

    // Measurement
    Z_lidar_ = Eigen::VectorXd::Zero(dim_Z_lidar_);
    Z_camera_ = Eigen::VectorXd::Zero(dim_Z_camera_);
    Z_lead_ = Eigen::VectorXd::Zero(dim_Z_lead_);

    // Absolute Pose of follow vehicle and lead vehicle
    abs_pose_follow_CG_ = Eigen::Vector3d::Zero(); 
    abs_pose_follow_R_ = Eigen::Vector3d::Zero(); 

    time_ = 0.0;
    last_obs_time_ = 0.0;
}

void RelPoseEstimator::Run(std::deque<IMUVelocitySideslipData> &follow_data_buff,
                           std::deque<IMUVelocitySideslipData> &lead_data_buff,
                           std::deque<LidarRelPose> &lidar_rel_pose_data_buff,
                           std::deque<CameraRelPose> &camera_rel_pose_data_buff)
{
    /* 1. Read Input and Measurement from Data Buffer */
    if(!follow_data_buff.empty())
    {
        IMUVelocitySideslipData current_follow_data = follow_data_buff.back();
        delta_t_ = current_follow_data.timestamp_ - time_;
        time_ = current_follow_data.timestamp_;

        vx1_ = current_follow_data.vx_;
        wz1_ = current_follow_data.angular_velocity_.z;
        ax1_ = current_follow_data.linear_acceleration_.x;
        beta1_ = current_follow_data.sideslip_angle_;
    }

    if(!lead_data_buff.empty())
    {
        IMUVelocitySideslipData current_lead_data = lead_data_buff.back();
        double vx2 = current_lead_data.vx_;
        double wz2 = current_lead_data.angular_velocity_.z;
        double ax2 = current_lead_data.linear_acceleration_.x;
        double beta2 = current_lead_data.sideslip_angle_;

        Z_lead_ << vx2, wz2, ax2, beta2;
        last_obs_time_ = current_lead_data.timestamp_;
    }

    if(!lidar_rel_pose_data_buff.empty())
    {
        LidarRelPose current_lidar_data = lidar_rel_pose_data_buff.back();
        Z_lidar_ = Eigen::Vector3d(current_lidar_data.rel_pose_.dx, 
                                   current_lidar_data.rel_pose_.dy,
                                   current_lidar_data.rel_pose_.dphi);
        last_obs_time_ = current_lidar_data.timestamp_;
    }

    if(!camera_rel_pose_data_buff.empty())
    {
        CameraRelPose current_camera_data = camera_rel_pose_data_buff.back();
        Z_camera_ = Eigen::Vector3d(current_camera_data.rel_pose_.dx, 
                                    current_camera_data.rel_pose_.dy,
                                    current_camera_data.rel_pose_.dphi);
        last_obs_time_ = current_camera_data.timestamp_;
    }

    /* 2. Initialize UKF */
    if(!is_initialized_)
    {
        delta_t_ = 0.01;
        ReadConfig();
        
        if(!lidar_rel_pose_data_buff.empty() || !camera_rel_pose_data_buff.empty())
        {
            Init(lidar_rel_pose_data_buff, camera_rel_pose_data_buff);
        }
        
        if (is_initialized_)
        {
            std::cout << "Rel Pose Estimator Initialization succeeds !" << std::endl;
            std::cout << "Rel Pose Estimation starts........" << std::endl;
        }
        else
        {
            return;
        }
    }
    
    if((time_ - last_obs_time_ > 0.5) && is_initialized_)  
    {
        // We don't run UKF, if we don't get any observation from Lidar, Camera or V2V in more than 0.5 second.
        std::cout << "[W]: " << time_
                             << "\t      --->  Warning: The relative Pose can not be estimated !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!" << std::endl;
        return;
    }

    /* 3. Run UKF for Relative Pose Estimation */
    GenerateSigmaPoints();
    Predict();

    if(!lead_data_buff.empty())
        UpdateLeadStates();
    
    if(!lidar_rel_pose_data_buff.empty())
        UpdateLidar();
    
    if(!camera_rel_pose_data_buff.empty())
        UpdateCamera(camera_rel_pose_data_buff.back().num_detected_aruco_markers);


    /* 4. Optional: Calculate Absolute Pose of follow vehicle */
    ComputeAbsPose();
}

void RelPoseEstimator::ReadConfig()
{
    YAML::Node config = YAML::LoadFile("src/sensorfusion/config/rel_pose_estimator_config.yaml");
    
    // Process noise variance
    rel_params_.ax_process_noise_var = config["RelPoseEstimator"]["ax_process_noise_var"].as<double>();
    rel_params_.wz_dot_process_noise_var = config["RelPoseEstimator"]["wz_dot_process_noise_var"].as<double>();
    rel_params_.beta_dot_process_noise_var = config["RelPoseEstimator"]["beta_dot_process_noise_var"].as<double>();

    // Measurement noise variance
    rel_params_.lidar_dx_noise_var = config["RelPoseEstimator"]["lidar_dx_noise_var"].as<double>();
    rel_params_.lidar_dy_noise_var = config["RelPoseEstimator"]["lidar_dy_noise_var"].as<double>();
    rel_params_.lidar_dphi_noise_var = config["RelPoseEstimator"]["lidar_dphi_noise_var"].as<double>();

    rel_params_.camera_dx_noise_var = config["RelPoseEstimator"]["camera_dx_noise_var"].as<double>();
    rel_params_.camera_dy_noise_var = config["RelPoseEstimator"]["camera_dy_noise_var"].as<double>();
    rel_params_.camera_dphi_noise_var = config["RelPoseEstimator"]["camera_dphi_noise_var"].as<double>();

    rel_params_.vx_estimated_noise_var = config["RelPoseEstimator"]["vx_estimated_noise_var"].as<double>();
    rel_params_.wz_IMU_noise_var = config["RelPoseEstimator"]["wz_IMU_noise_var"].as<double>();
    rel_params_.ax_IMU_noise_var = config["RelPoseEstimator"]["ax_IMU_noise_var"].as<double>();
    rel_params_.beta_estimated_noise_var = config["RelPoseEstimator"]["beta_estimated_noise_var"].as<double>();

    // Initial value of states variance
    std::vector<double> P0_diag = config["RelPoseEstimator"]["P0_diag"].as<std::vector<double>>();
    rel_params_.P0 = Eigen::MatrixXd::Identity(dim_X_, dim_X_);
    for(int i = 0; i < dim_X_; ++i)
    {
        rel_params_.P0(i, i) = P0_diag[i];
    }


    // UKF Tuning Parameters
    alpha_ = config["RelPoseEstimator"]["alpha"].as<double>();
    kappa_ = config["RelPoseEstimator"]["kappa"].as<double>();
    rho_ = config["RelPoseEstimator"]["rho"].as<double>();
}

void RelPoseEstimator::Init(std::deque<LidarRelPose> &lidar_rel_pose_data_buff,
                            std::deque<CameraRelPose> &camera_rel_pose_data_buff)
{
    // Initialize absolute pose of follow vehicle (w.r.t. Center of Gravity) using global pose (w.r.t. vhicle rear (Origin Fr1)) from CarMaker
    abs_pose_follow_CG_ = tools::TransformR2CG(groundtruth_follow_abs_pose);

    // Initialize state vector using observations
    if (!lidar_rel_pose_data_buff.empty())
    {
        X_ << Z_lidar_, Z_lead_;
    }
    else if (!camera_rel_pose_data_buff.empty())
    {
        X_ << Z_camera_, Z_lead_;
    }
    else
    {
        // Initialization will continue until a valid observation from lidar or camera is received.
        return;
    }

    // Initialize augmented state vector
    X_aug_ << X_, Eigen::VectorXd::Zero(dim_Nu_);

    // Initialize process noise covariance and measurement noise covariance matrix
    process_noise_cov_(0, 0) = rel_params_.ax_process_noise_var;
    process_noise_cov_(1, 1) = rel_params_.wz_dot_process_noise_var;
    process_noise_cov_(2, 2) = rel_params_.beta_dot_process_noise_var;

    lidar_noise_cov_(0, 0) = rel_params_.lidar_dx_noise_var;
    lidar_noise_cov_(1, 1) = rel_params_.lidar_dy_noise_var;
    lidar_noise_cov_(2, 2) = rel_params_.lidar_dphi_noise_var;

    camera_noise_cov_(0, 0) = rel_params_.camera_dx_noise_var;
    camera_noise_cov_(1, 1) = rel_params_.camera_dy_noise_var;
    camera_noise_cov_(2, 2) = rel_params_.camera_dphi_noise_var;

    lead_noise_cov_(0, 0) = rel_params_.vx_estimated_noise_var;
    lead_noise_cov_(1, 1) = rel_params_.wz_IMU_noise_var;
    lead_noise_cov_(2, 2) = rel_params_.ax_IMU_noise_var;
    lead_noise_cov_(3, 3) = rel_params_.beta_estimated_noise_var;


    // Initialize state covariance matrix
    P_ = rel_params_.P0;

    // Initialize augmented state covariance matrix
    P_aug_.block(0, 0, dim_X_, dim_X_) = P_;
    P_aug_.block(dim_X_, dim_X_, dim_Nu_, dim_Nu_) = process_noise_cov_;

    // set UKF tuning parameter: lambda
    lambda_ = alpha_ * alpha_ * (L_ + kappa_) - L_;                      

    // set weights
    weights_m_.reserve(2 * L_ + 1);
    weights_c_.reserve(2 * L_ + 1);
    weights_m_[0] = lambda_ / (L_ + lambda_);
    weights_c_[0] = lambda_ / (L_ + lambda_) + (1 - alpha_ * alpha_ + rho_);
    double w = 1 / (2 * (L_ + lambda_));
    for(int i = 1; i < 2 * L_ + 1; ++i)
    {
        weights_m_[i] = w;
        weights_c_[i] = w;
    }

    is_initialized_ = true;
}

void RelPoseEstimator::GenerateSigmaPoints() 
{
    // Update augmented state and augmented state covariance matrix, w.r.t. the current state and state covariance matrix
    X_aug_.setZero();
    X_aug_ << X_, Eigen::VectorXd::Zero(dim_Nu_);
    P_aug_.setZero();
    P_aug_.block(0, 0, dim_X_, dim_X_) = P_;
    P_aug_.block(dim_X_, dim_X_, dim_Nu_, dim_Nu_) = process_noise_cov_;

    // Calculate lower triangular matrix
    Eigen::MatrixXd lower_triangular = P_aug_.llt().matrixL();
    lower_triangular *= std::sqrt(lambda_ + L_);

    // Generate augmented sigma points matrix
    Sigma_Points_aug_.setZero();
    Sigma_Points_aug_.col(0) = X_aug_;
    for (int i = 0; i < L_; ++i) {
        Sigma_Points_aug_.col(i + 1) = X_aug_ + lower_triangular.col(i);
        Sigma_Points_aug_.col(i + L_ + 1) = X_aug_ - lower_triangular.col(i);
    }
}



void RelPoseEstimator::Predict()
{
    // 1. Predict Sigma Points
    double dt2 = delta_t_ * delta_t_;

    double wz1_dt = wz1_ * delta_t_;
    double cos_wz1_dt = std::cos(wz1_dt);
    double sin_wz1_dt = std::sin(wz1_dt);

    double v1_dt = vx1_ * std::sqrt(1 + std::pow(std::tan(beta1_),2)) * delta_t_;

    double wz1_beta1 = 0.5 * wz1_dt + beta1_;
    double cos_wz1_beta1 = std::cos(wz1_beta1);
    double sin_wz1_beta1 = std::sin(wz1_beta1);
    double longi_1 = v1_dt * cos_wz1_beta1;
    double later_1 = v1_dt * sin_wz1_beta1;

    for(int i = 0; i < 2 * L_ + 1; ++i)
    {
        /* a. Read states and noise from Sigma Points */
        double dx = Sigma_Points_aug_(0, i);
        double dy = Sigma_Points_aug_(1, i);
        double dphi = Sigma_Points_aug_(2, i);
        dphi = tools::NormalizeAngle2(dphi);
        double vx2 = Sigma_Points_aug_(3, i);
        double wz2 = Sigma_Points_aug_(4, i);
        double ax2 = Sigma_Points_aug_(5, i);
        double beta2 = Sigma_Points_aug_(6, i);
        beta2 = tools::NormalizeAngle2(beta2);

        double nu_ax = Sigma_Points_aug_(7, i);
        double nu_wz_dot = Sigma_Points_aug_(8, i);
        double nu_beta_dot = Sigma_Points_aug_(9, i);

        /* b. Transform the relative pose from RF to CGs */
        Eigen::Vector3d rel_pose_RF = Eigen::Vector3d(dx, dy, dphi);
        Eigen::Vector3d rel_pose_CGs = tools::TransformRF2CGs(rel_pose_RF, dphi);
        dx = rel_pose_CGs(0);
        dy = rel_pose_CGs(1);

        /* c. State transition for  dx, dy */
        double vx2_dot = ax2 + wz2 * vx2 * std::tan(beta2);
        double v2_dt = vx2 * std::sqrt(1 + std::pow(std::tan(beta2),2)) * delta_t_;

        double dphi_wz2_beta2 = dphi + 0.5 * wz2 * delta_t_ + beta2;

        double cos_dphi_wz2_beta2 = std::cos(dphi_wz2_beta2);
        double sin_dphi_wz2_beta2 = std::sin(dphi_wz2_beta2);
        double longi_2 = v2_dt * cos_dphi_wz2_beta2;
        double later_2 = v2_dt * sin_dphi_wz2_beta2;

        double dx_temp = dx + longi_2 - longi_1;
        double dy_temp = dy + later_2 - later_1;

        // coordinate transform from "0 degree" to "wz1_dt" 
        double dx_pred = cos_wz1_dt * dx_temp + sin_wz1_dt * dy_temp;
        double dy_pred = - sin_wz1_dt * dx_temp + cos_wz1_dt * dy_temp;

        /* d. State transition for  dphi, vx2, wz2, ax2, beta2 */
        double dphi_pred = dphi + (wz2 - wz1_) * delta_t_;
        dphi_pred = tools::NormalizeAngle2(dphi_pred);
        double vx2_pred = vx2 + vx2_dot * delta_t_;
        double wz2_pred = wz2;
        double ax2_pred = ax2;
        double beta2_pred = beta2;

        /* e. Add process noise to state */
        double nx_ax_dt2_half = nu_ax * dt2 / 2;
        dx_pred += nx_ax_dt2_half * std::cos(dphi) + nx_ax_dt2_half;
        dy_pred += nx_ax_dt2_half * std::sin(dphi);
        dphi_pred += nu_wz_dot * dt2;
        vx2_pred += nu_ax * delta_t_;
        wz2_pred += nu_wz_dot * delta_t_ * 0.5;
        ax2_pred += nu_ax * 0.1;
        beta2_pred += nu_beta_dot * delta_t_ * 0.5;

        /* f. Transform the relative pose from RF to CGs */
        Eigen::Vector3d rel_pose_CGs_pred = Eigen::Vector3d(dx_pred, dy_pred, dphi_pred);
        Eigen::Vector3d rel_pose_RF_pred = tools::TransformCGs2RF(rel_pose_CGs_pred);

        /* g. Save predicted sigma point to sigma point matrix */
        Sigma_Points_(0, i) = rel_pose_RF_pred(0);
        Sigma_Points_(1, i) = rel_pose_RF_pred(1);
        Sigma_Points_(2, i) = dphi_pred;
        Sigma_Points_(3, i) = vx2_pred;
        Sigma_Points_(4, i) = wz2_pred;
        Sigma_Points_(5, i) = ax2_pred;
        Sigma_Points_(6, i) = beta2_pred;
    }

    // 2. Compute gaussian statistics (mean and covariance) of predicted Sigma Points
    X_.setZero();
    for(int i = 0; i < 2 * L_ + 1; ++i)
    {
        X_ += weights_m_[i] * Sigma_Points_.col(i);
    }
    X_(2) = tools::NormalizeAngle2(X_(2));

    P_.setZero();
    for(int i = 0; i < 2 * L_ + 1; ++i)
    {
        Eigen::VectorXd X_bias = Sigma_Points_.col(i) - X_;
        X_bias(2) = tools::NormalizeAngle2(X_bias(2));
        X_bias(6) = tools::NormalizeAngle(X_bias(6));
        P_ += weights_c_[i] * X_bias * X_bias.transpose();
    }
}

void RelPoseEstimator::UpdateLidar()
{
    // Compute predicted virtual measurement at sigma points
    // Sigma_Points_Z is a matrix for storing the corresponding predicted virtual measurement at sigma points
    Eigen::MatrixXd Sigma_Points_Z = Eigen::MatrixXd::Zero(dim_Z_lidar_, 2 * L_ + 1);   
    for(int i = 0; i < 2 * L_ + 1; ++i)
    {
        Sigma_Points_Z(0, i) = Sigma_Points_(0, i);         // dx
        Sigma_Points_Z(1, i) = Sigma_Points_(1, i);         // dy
        Sigma_Points_Z(2, i) = Sigma_Points_(2, i);         // dphi
    }

    Update(Z_lidar_, Sigma_Points_Z, lidar_noise_cov_);
}


void RelPoseEstimator::UpdateCamera(int num_detected_aruco_markers)
{
    // Compute predicted virtual measurement at sigma points
    Eigen::MatrixXd Sigma_Points_Z = Eigen::MatrixXd::Zero(dim_Z_camera_, 2 * L_ + 1);   
    for(int i = 0; i < 2 * L_ + 1; ++i)
    {
        Sigma_Points_Z(0, i) = Sigma_Points_(0, i);         // dx
        Sigma_Points_Z(1, i) = Sigma_Points_(1, i);         // dy
        Sigma_Points_Z(2, i) = Sigma_Points_(2, i);         // dphi
    }

    Eigen::MatrixXd camera_noise_cov = camera_noise_cov_;

    // The relative pose detected by camera tracker has a bigger noise covariance, if the number of detected arruco markers is less.
    // For OBTS Aruco Markers Concept: num_detected_aruco_markers = 1, 2 or 3   (0 was invalid and excluded)   
    // if(num_detected_aruco_markers == 1)
    // {
    //     camera_noise_cov *= 5;
    // }

    // else if(num_detected_aruco_markers == 2)
    // {
    //     camera_noise_cov(0,0) *= 2;
    //     camera_noise_cov(2,2) *= 3;
    // }
    // else {}

    if(num_detected_aruco_markers == 2)
    {
        camera_noise_cov(0,0) *= 2;
        camera_noise_cov(2,2) *= 3;
    }
        
    Update(Z_camera_, Sigma_Points_Z, camera_noise_cov);
}


void RelPoseEstimator::UpdateLeadStates()
{
    // Compute predicted virtual measurement at sigma points
    // Sigma_Points_Z is a matrix for storing the corresponding predicted virtual measurement at sigma points
    Eigen::MatrixXd Sigma_Points_Z = Eigen::MatrixXd::Zero(dim_Z_lead_, 2 * L_ + 1);   
    for(int i = 0; i < 2 * L_ + 1; ++i)
    {
        Sigma_Points_Z(0, i) = Sigma_Points_(3, i);         // vx2
        Sigma_Points_Z(1, i) = Sigma_Points_(4, i);         // wz2
        Sigma_Points_Z(2, i) = Sigma_Points_(5, i);         // ax2
        Sigma_Points_Z(3, i) = Sigma_Points_(6, i);         // beta2
    }

    Eigen::VectorXd last_pose  = X_.segment(0,3);
    Eigen::MatrixXd last_pose_P = P_.block(0,0,3,3);
    Update(Z_lead_, Sigma_Points_Z, lead_noise_cov_);  
}

void RelPoseEstimator::Update(const Eigen::VectorXd &Z, const Eigen::MatrixXd &Sigma_Points_Z, const Eigen::MatrixXd &measurement_noise_cov)
{
    // 1. Compute predicted virtual measurement mean
    int dim_Z = Z.rows();
    Eigen::VectorXd Z_virtual = Eigen::VectorXd::Zero(dim_Z);
    for(int i = 0; i < 2 * L_ + 1; ++i)
    {
        Z_virtual += weights_m_[i] * Sigma_Points_Z.col(i);
    }

    // 2. Compute covariance of virtual measurement
    Eigen::MatrixXd P_Z_virtual = Eigen::MatrixXd::Zero(dim_Z, dim_Z);
    for(int i = 0; i < 2 * L_ + 1; ++i)
    {
        Eigen::VectorXd Z_virtual_bais = Sigma_Points_Z.col(i) - Z_virtual;
        P_Z_virtual += weights_c_[i] * Z_virtual_bais * Z_virtual_bais.transpose();
    }
    P_Z_virtual += measurement_noise_cov;               // We have additive measurement noise

    // 3. Compute cross covariance of predicted state and virtual measurement
    Eigen::MatrixXd P_X_Z_virtual = Eigen::MatrixXd::Zero(dim_X_, dim_Z);
    for(int i = 0; i < 2 * L_ + 1; ++i)
    {
        Eigen::VectorXd X_bias = Sigma_Points_.col(i) - X_;
        X_bias(2) = tools::NormalizeAngle2(X_bias(2));
        X_bias(6) = tools::NormalizeAngle(X_bias(6));
        Eigen::VectorXd Z_virtual_bias = Sigma_Points_Z.col(i) - Z_virtual;
        P_X_Z_virtual += weights_c_[i] * X_bias * Z_virtual_bias.transpose();
    }

    // 4. Compute kalman gain
    Eigen::MatrixXd K = P_X_Z_virtual * P_Z_virtual.inverse();

    // 5. Update state mean and state covariance matrix
    X_ += K * (Z - Z_virtual);
    X_(2) = tools::NormalizeAngle2(X_(2));
    X_(6) = tools::NormalizeAngle(X_(6));
    P_ -= K * P_Z_virtual * K.transpose();
}


void RelPoseEstimator::ComputeAbsPose()
{
    double dt2 = delta_t_ * delta_t_;

    // Compute absolute Pose of follow vehicle
    double phi1 = abs_pose_follow_CG_(2);
    double phi1_wz1_beta1 = phi1 + 0.5 * wz1_ * delta_t_ + beta1_;
    double cos_phi1_wz1_beta1 = std::cos(phi1_wz1_beta1);
    double sin_phi1_wz1_beta1 = std::sin(phi1_wz1_beta1);

    double vx1_dot = ax1_ + wz1_ * vx1_ * std::tan(beta1_);
    double vx1_ax1_dt = vx1_ * delta_t_ + 0.5 * vx1_dot * dt2;

    double follow_longi_move = vx1_ax1_dt * cos_phi1_wz1_beta1;
    double follow_later_move = vx1_ax1_dt * sin_phi1_wz1_beta1;

    abs_pose_follow_CG_(0) += follow_longi_move;
    abs_pose_follow_CG_(1) += follow_later_move;
    abs_pose_follow_CG_(2) += wz1_ * delta_t_;

    abs_pose_follow_R_ = tools::TransformCG2R(abs_pose_follow_CG_);
}

Eigen::Vector3d RelPoseEstimator::GetRelPose()
{
    // return [dx, dy, dphi] 
    return X_.head(3);       
}

Eigen::Vector3d RelPoseEstimator::GetAbsPoseFollow()
{
    // return [x1, y1, phi1] 
    return abs_pose_follow_R_;
}

Eigen::Vector4d RelPoseEstimator::GetLeadStates()
{
    // return [vx2, wz2, ax2, beta2]
    return X_.tail(4);
}

void RelPoseEstimator::GetStatesStandardDeviation(Eigen::VectorXd &states_stdd)
{
    states_stdd = Eigen::VectorXd(7);
    states_stdd(0) = std::sqrt(P_(0,0));
    states_stdd(1) = std::sqrt(P_(1,1));
    states_stdd(2) = std::sqrt(P_(2,2));
    states_stdd(3) = std::sqrt(P_(3,3));
    states_stdd(4) = std::sqrt(P_(4,4));
    states_stdd(5) = std::sqrt(P_(5,5));
    states_stdd(6) = std::sqrt(P_(6,6));
}
