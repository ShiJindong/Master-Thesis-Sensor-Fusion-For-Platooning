#include "headers/sideslip_angle_estimator.h"
#include "../User.h"

SideslipAngleEstimator::SideslipAngleEstimator()
{
    is_initialized_ = false;
    delta_t_ = 0;
    
    dim_X_ = 3;
    dim_Nu_ = 3;
    dim_Z_ = 3;
    L_ = dim_X_ + dim_Nu_;

    X_ = Eigen::VectorXd::Zero(dim_X_);
    X_aug_ = Eigen::VectorXd::Zero(L_);

    process_noise_cov_ = Eigen::MatrixXd::Zero(dim_Nu_, dim_Nu_);
    measurement_noise_cov_ = Eigen::MatrixXd::Zero(dim_Z_, dim_Z_);

    P_ = Eigen::MatrixXd::Zero(dim_X_, dim_X_);
    P_aug_ = Eigen::MatrixXd::Zero(L_, L_);

    Z_ = Eigen::VectorXd::Zero(dim_Z_);

    Sigma_Points_ = Eigen::MatrixXd::Zero(dim_X_, 2 * L_ + 1);
    Sigma_Points_aug_ = Eigen::MatrixXd::Zero(L_, 2 * L_ + 1);

    ax_IMU_ = 0; 
    steer_angle_ = 0;

    ay_IMU_ = 0; 
    vx_estimated_ = 0;
    wz_IMU_ = 0;
    
    sideslip_angle_ = 0;
}

void SideslipAngleEstimator::Run(std::deque<WheelSteerAngleData> &wheel_steer_angle_data_buff, 
                                 std::deque<IMUData> &imu_data_buff, 
                                 double vx_estimated)
{
    SubscribeIMUData(imu_data_buff, imu_data_buff_);
    SubscribeWheelSteerAngleData(wheel_steer_angle_data_buff);
    vx_estimated_ = vx_estimated;
    
    /* 1. Initialize UKF */
    if(!is_initialized_ && HasIMUData())
    {
        ReadConfig();
        if(ValidData())
        {
            Init();
            is_initialized_ = true;
        }
        return;
    }

    /* 2. Run UKF for State Estimation */
    while(HasIMUData())
    {
        if(ValidData())
        {
            GenerateSigmaPoints();
            Predict();
            Update();
        }
    }

    /* 3. Calculate Vehicle Sideslip Angle using estimated vx, vy */
    CalculateSideslipAngle();
}

bool SideslipAngleEstimator::ValidData()
{
    // Read Input and Measurement from Data Buffer
    ax_IMU_ = current_imu_data_.linear_acceleration_.x;
    ay_IMU_ = current_imu_data_.linear_acceleration_.y;
    wz_IMU_ = current_imu_data_.angular_velocity_.z;
    steer_angle_ = (current_wheel_steer_angle_data_.wheel_steer_angle_(0) +
                    current_wheel_steer_angle_data_.wheel_steer_angle_(1)) / 2.0;

    if(std::isnan(ax_IMU_) || std::isnan(ay_IMU_) || std::isnan(wz_IMU_) || std::isnan(steer_angle_))
    {
        return false;
    } 

    return true;  
}

void SideslipAngleEstimator::ReadConfig()
{
    YAML::Node config_1 = YAML::LoadFile("src/sensorfusion/config/vehicle_parameters_config.yaml");
    sideslip_params_.m = config_1["VehicleParamters"]["m"].as<double>();
    sideslip_params_.Iz = config_1["VehicleParamters"]["Iz"].as<double>();
    sideslip_params_.lf = config_1["VehicleParamters"]["lf"].as<double>();
    sideslip_params_.lr = config_1["VehicleParamters"]["lr"].as<double>();
    sideslip_params_.Cf = config_1["VehicleParamters"]["Cf"].as<double>();
    sideslip_params_.Cr = config_1["VehicleParamters"]["Cr"].as<double>();



    YAML::Node config_2 = YAML::LoadFile("src/sensorfusion/config/sideslip_angle_estimator_config.yaml");
    sideslip_params_.ax_process_noise_var = config_2["SideslipAngleEstimator"]["ax_process_noise_var"].as<double>();
    sideslip_params_.ay_process_noise_var = config_2["SideslipAngleEstimator"]["ay_process_noise_var"].as<double>();
    sideslip_params_.wz_dot_process_noise_var = config_2["SideslipAngleEstimator"]["wz_dot_process_noise_var"].as<double>();
    sideslip_params_.ay_IMU_noise_var = config_2["SideslipAngleEstimator"]["ay_IMU_noise_var"].as<double>();
    sideslip_params_.vx_estimated_noise_var = config_2["SideslipAngleEstimator"]["vx_estimated_noise_var"].as<double>();
    sideslip_params_.wz_IMU_noise_var = config_2["SideslipAngleEstimator"]["wz_IMU_noise_var"].as<double>();

    sideslip_params_.vx_threshold = config_2["SideslipAngleEstimator"]["vx_threshold"].as<double>();

    alpha_ = config_2["SideslipAngleEstimator"]["alpha"].as<double>();
    kappa_ = config_2["SideslipAngleEstimator"]["kappa"].as<double>();
    rho_ = config_2["SideslipAngleEstimator"]["rho"].as<double>();

    std::vector<double> P0_vec = config_2["SideslipAngleEstimator"]["P0"].as<std::vector<double>>();
    sideslip_params_.P0 = Eigen::MatrixXd::Identity(dim_X_, dim_X_);
    for(Eigen::Index row = 0; row < sideslip_params_.P0.rows(); ++row)
    {
        for(Eigen::Index col = 0; col < sideslip_params_.P0.cols(); ++col)
        {
            sideslip_params_.P0(row, col) = P0_vec[row * sideslip_params_.P0.cols() + col];
        }
    }
}

void SideslipAngleEstimator::Init()
{
    // Initialize state vector
    X_ << vx_estimated_, 0, wz_IMU_;

    // Initialize augmented state vector
    X_aug_ << X_, Eigen::VectorXd::Zero(dim_Nu_);

    // Initialize process noise covariance and measurement noise covariance matrix
    process_noise_cov_(0, 0) = sideslip_params_.ax_process_noise_var;
    process_noise_cov_(1, 1) = sideslip_params_.ay_process_noise_var;
    process_noise_cov_(2, 2) = sideslip_params_.wz_dot_process_noise_var;
    measurement_noise_cov_(0, 0) = sideslip_params_.ay_IMU_noise_var;
    measurement_noise_cov_(1, 1) = sideslip_params_.vx_estimated_noise_var;
    measurement_noise_cov_(2, 2) = sideslip_params_.wz_IMU_noise_var;

    // Initialize state covariance matrix
    P_ = sideslip_params_.P0;

    // Initialize augmented state covariance matrix
    P_aug_.block(0, 0, dim_X_, dim_X_) = P_;
    P_aug_.block(dim_X_, dim_X_, dim_Nu_, dim_Nu_) = process_noise_cov_;

    // set UKF tuning parameter: lambda
    lambda_ = alpha_ * alpha_ * (L_ + kappa_) - L_;     
    // in some papers there is another suggestion:  lambda_ = 3 - L_;
    // lambda_ = 3 - L_;                  

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
}

void SideslipAngleEstimator::GenerateSigmaPoints() 
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

#if DEBUG
    std::cout << "-----------------------------------------------------------------------------------------------------" << std::endl;
    std::cout << "X_aug = " << X_aug_.transpose() << std::endl;
    std::cout << "P_aug = \n" << P_aug_ << std::endl;
    std::cout << "lower_triangular = \n" << lower_triangular << std::endl;
    std::cout << "Sigma_Points_aug_ = \n" << Sigma_Points_aug_ << std::endl;
#endif
}

void SideslipAngleEstimator::Predict()
{
    
#if DEBUG
    std::cout << "(Before Prediction) Sigma_Points_ = \n" << Sigma_Points_ << std::endl;
#endif

    // 1. Predict Sigma Points
    delta_t_ = current_imu_data_.timestamp_ - time_;
    time_ = current_imu_data_.timestamp_;

    if(delta_t_ > 1.0)
    {
        return;
    }

    double cos_steer_angle = std::cos(steer_angle_);
    for(int i = 0; i < 2 * L_ + 1; ++i)
    {
        double vx = Sigma_Points_aug_(0, i);
        double vy = Sigma_Points_aug_(1, i);
        double wz = Sigma_Points_aug_(2, i);
        double nu_ax = Sigma_Points_aug_(3, i);
        double nu_ay = Sigma_Points_aug_(4, i);
        double nu_wz_dot = Sigma_Points_aug_(5, i);

        // State transition for 1. State: vx
        double vx_dot = ax_IMU_ + wz * vy;
        double vx_pred = vx + vx_dot * delta_t_;

        // State transition for 2. State: vy
        double alpha_f = 0, alpha_r = 0;                        // Estimated tire slip angle of front and rear tires
        if(std::fabs(vx) > sideslip_params_.vx_threshold)
        {
            alpha_f = steer_angle_ - (vy + sideslip_params_.lf * wz) / vx;
            alpha_r = (sideslip_params_.lr * wz - vy) / vx;
        }

        double Fyf = 2 * sideslip_params_.Cf * alpha_f;        // Estimated lateral force of front tires
        double Fyr = 2 * sideslip_params_.Cr * alpha_r;        // Estimated lateral force of rear tires
        double vy_dot = (Fyf * cos_steer_angle + Fyr) / sideslip_params_.m - wz * vx;
        double vy_pred = vy + vy_dot * delta_t_;

        // State transition for 3. State: wz
        double wz_dot = (Fyf * sideslip_params_.lf * cos_steer_angle - Fyr * sideslip_params_.lr) / sideslip_params_.Iz;
        double wz_pred = wz + wz_dot * delta_t_;

        // Add process noise to state
        vx_pred += nu_ax * delta_t_;
        vy_pred += nu_ay * delta_t_;
        wz_pred += nu_wz_dot * delta_t_;

        // save predicted sigma point to sigma point matrix
        Sigma_Points_(0, i) = vx_pred;
        Sigma_Points_(1, i) = vy_pred;
        Sigma_Points_(2, i) = wz_pred;
    }

#if DEBUG
    std::cout << "(After Prediction) Sigma_Points_ = \n" << Sigma_Points_ << std::endl;
#endif

    // 2. Compute gaussian statistics (mean and covariance) of predicted Sigma Points
    X_.setZero();
    for(int i = 0; i < 2 * L_ + 1; ++i)
    {
        X_ += weights_m_[i] * Sigma_Points_.col(i);
    }

    P_.setZero();
    for(int i = 0; i < 2 * L_ + 1; ++i)
    {
        Eigen::VectorXd X_bias = Sigma_Points_.col(i) - X_;
        P_ += weights_c_[i] * X_bias * X_bias.transpose();
    }

#if DEBUG
    std::cout << "X = " << X_.transpose() << std::endl;
    std::cout << "P = \n" << P_ << std::endl;
#endif
}

void SideslipAngleEstimator::Update()
{
    // 1. Compute predicted virtual measurement at sigma points
    // Sigma_Points_Z is a matrix for storing the corresponding predicted virtual measurement at sigma points
    Eigen::MatrixXd Sigma_Points_Z = Eigen::MatrixXd::Zero(dim_Z_, 2 * L_ + 1);   
    for(int i = 0; i < 2 * L_ + 1; ++i)
    {
        double vx = Sigma_Points_(0, i);
        double vy = Sigma_Points_(1, i);
        double wz = Sigma_Points_(2, i);

        double alpha_f = 0, alpha_r = 0;                            // Estimated tire slip angle of front and rear tires
        if(std::fabs(vx) > sideslip_params_.vx_threshold)
        {
            alpha_f = steer_angle_ - (vy + sideslip_params_.lf * wz) / vx;
            alpha_r = (sideslip_params_.lr * wz - vy) / vx;
        }
        double Fyf = 2 * sideslip_params_.Cf * alpha_f;            // Estimated lateral force of front tires
        double Fyr = 2 * sideslip_params_.Cr * alpha_r;            // Estimated lateral force of rear tires

        Sigma_Points_Z(0, i) = (Fyf + Fyr) / sideslip_params_.m;   // virtual measurement for ay
        Sigma_Points_Z(1, i) = vx;
        Sigma_Points_Z(2, i) = wz;
    }

    // 2. Compute predicted virtual measurement mean
    Eigen::VectorXd Z_virtual = Eigen::VectorXd::Zero(dim_Z_);
    for(int i = 0; i < 2 * L_ + 1; ++i)
    {
        Z_virtual += weights_m_[i] * Sigma_Points_Z.col(i);
    }

    // 3. Compute covariance of virtual measurement
    Eigen::MatrixXd P_Z_virtual = Eigen::MatrixXd::Zero(dim_Z_, dim_Z_);
    for(int i = 0; i < 2 * L_ + 1; ++i)
    {
        Eigen::VectorXd Z_virtual_bais = Sigma_Points_Z.col(i) - Z_virtual;
        P_Z_virtual += weights_c_[i] * Z_virtual_bais * Z_virtual_bais.transpose();
    }
    P_Z_virtual += measurement_noise_cov_;    // We have additive measurement noise

    // 4. Compute cross covariance of predicted state and virtual measurement
    Eigen::MatrixXd P_X_Z_virtual = Eigen::MatrixXd::Zero(dim_X_, dim_Z_);
    for(int i = 0; i < 2 * L_ + 1; ++i)
    {
        Eigen::VectorXd X_bias = Sigma_Points_.col(i) - X_;
        Eigen::VectorXd Z_virtual_bias = Sigma_Points_Z.col(i) - Z_virtual;
        P_X_Z_virtual += weights_c_[i] * X_bias * Z_virtual_bias.transpose();
    }

    // 5. Compute kalman gain
    Eigen::MatrixXd K = P_X_Z_virtual * P_Z_virtual.inverse();

    // 6. Update state mean and state covariance matrix (using caluclated kalman gain K and obtained measurement vector Z_)
    Z_ << ay_IMU_, vx_estimated_, wz_IMU_;    
    X_ += K * (Z_ - Z_virtual);
    P_ -= K * P_Z_virtual * K.transpose();
}

void SideslipAngleEstimator::CalculateSideslipAngle()
{
    double vx = X_(0);
    double vy = X_(1);

    if(std::fabs(vx) > sideslip_params_.vx_threshold)
    {
        sideslip_angle_ = std::atan(vy / vx);               // Vechile sideslip angle in the range (-PI/2, PI/2]
        // sideslip_angle_ = std::atan2(vy, vx);            // Vehicle sideslip angle in the range (-PI, PI]
    }
    else
    {
        sideslip_angle_ = 0.0;
    }
}

double SideslipAngleEstimator::GetSideslipAngle()
{
    return sideslip_angle_;
}

double SideslipAngleEstimator::GetLateralVelocity()
{
    return X_(1);
}

void SideslipAngleEstimator::SubscribeIMUData(std::deque<IMUData> &origin_imu_data_buff, 
                                              std::deque<IMUData> &subscribed_imu_data_buff)
{
    if (origin_imu_data_buff.size() > 0) {
        subscribed_imu_data_buff.insert(subscribed_imu_data_buff.end(), origin_imu_data_buff.begin(), origin_imu_data_buff.end());
    }
}

    
void SideslipAngleEstimator::SubscribeWheelSteerAngleData(std::deque<WheelSteerAngleData> &origin_wheel_steer_angle_data_buff)
{
    if(!origin_wheel_steer_angle_data_buff.empty())
    {
        current_wheel_steer_angle_data_ = origin_wheel_steer_angle_data_buff.back();
    }
}

bool SideslipAngleEstimator::HasIMUData()
{
    if (!imu_data_buff_.empty()) 
    {
        current_imu_data_ = imu_data_buff_.front();
        imu_data_buff_.pop_front();
        return true;
    }

    return false;
}