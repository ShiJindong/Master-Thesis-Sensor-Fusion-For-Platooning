#include "headers/velocity_estimator.h"
#include "../User.h"

VelocityEstimator::VelocityEstimator()
{
    is_initialized_ = false; 
    delta_t_ = 0;

    ax_ = 0;
    wz_ = 0;
    beta_ = 0;                     
    vx_best_ = 0; 
    sideslip_angle_estimated_ = 0;

    velocity_kf_.SetState(Eigen::Vector2d::Zero());

    X_prior_ = Eigen::Vector2d::Zero();
    P0_ = Eigen::Matrix2d::Zero();
    Q_ = Eigen::Matrix2d::Zero();
    R_ << 0;
    A_ = Eigen::Matrix2d::Zero();
    H_ = Eigen::Matrix<double, 1, 2>::Zero();

    is_at_least_one_wheel_data_valid_ = true;
    wheel_speed_CG_ = Eigen::Vector4d::Zero();

    init_velocity_ = 0.0;
}

void VelocityEstimator::Run(std::deque<IMUData> &imu_data_buff, 
                            std::deque<WheelRotRateData> &wheel_rot_rate_data_buff,   
                            std::deque<WheelSteerAngleData> &wheel_steer_angle_data_buff,  
                            double sideslip_angle_estimated,
                            std::deque<IMUVelocitySideslipData> &imu_velocity_sideslip_data_buff)
{
    SubscribeIMUData(imu_data_buff, imu_data_buff_);
    SubscribeWheelData(wheel_rot_rate_data_buff, wheel_steer_angle_data_buff, wheel_data_buff_);
    sideslip_angle_estimated_ = sideslip_angle_estimated;

    if(!is_initialized_)
    {
        Init();
        // wheel_slip_detector_.is_initialized_ = false;
        is_initialized_ = true;
    }

    while(HasIMUData())
    {
        if(HasWheelData())
        {
            if(current_imu_data_.timestamp_ < current_wheel_data_.timestamp_)
            {
                Predict(current_imu_data_);
                last_imu_data_ = current_imu_data_;              
            }
            else
            {
                double synch_time = current_wheel_data_.timestamp_;
                IMUData synch_imu_data;
                IMUData::SynchIMU(last_imu_data_, current_imu_data_, synch_time, synch_imu_data);
                Predict(synch_imu_data);
                Update(synch_imu_data, current_wheel_data_);
                Predict(current_imu_data_);

                last_imu_data_ = current_imu_data_;
                wheel_data_buff_.pop_front();
            }
        }
        else
        {
            Predict(current_imu_data_);
            last_imu_data_ = current_imu_data_;
        }

        // Save synchronised IMU, Velocity, Sideslip Angle Data into Data Buffer
        IMUVelocitySideslipData imu_velocity_sideslip_data_;
        imu_velocity_sideslip_data_.linear_acceleration_ = current_imu_data_.linear_acceleration_;
        imu_velocity_sideslip_data_.angular_velocity_ = current_imu_data_.angular_velocity_;
        imu_velocity_sideslip_data_.vx_ = velocity_kf_.GetState()(0);
        imu_velocity_sideslip_data_.vy_ = velocity_kf_.GetState()(0) * std::tan(sideslip_angle_estimated);
        imu_velocity_sideslip_data_.sideslip_angle_ = sideslip_angle_estimated;
        imu_velocity_sideslip_data_.timestamp_ = current_imu_data_.timestamp_;
        imu_velocity_sideslip_data_buff.push_back(imu_velocity_sideslip_data_);
    }

}

void VelocityEstimator::Predict(IMUData &imu_data)
{
    // Calculate time interval
    delta_t_ = imu_data.timestamp_ - time_;
    // Move timestamp forward
    time_ = imu_data.timestamp_;

    if(!std::isnan(imu_data.linear_acceleration_.x))
    {
        ax_ = imu_data.linear_acceleration_.x;
    }

    if(!std::isnan(imu_data.angular_velocity_.z))
    {
        wz_ = imu_data.angular_velocity_.z;
    }

    beta_ = sideslip_angle_estimated_;

    /* Prediction Step */
    double u = ax_ + velocity_kf_.GetState()(0) * std::tan(beta_) * wz_;
    A_ << 1, -g_ * delta_t_, 0, 1;


    /* We should give states in KF an initial value, if the V2V communication restarts again ! */
    if(delta_t_ > 1.0)
    {
        X_prior_ = Eigen::Vector2d(init_velocity_, 0);
    }
    else
    {
        X_prior_ = A_ * velocity_kf_.GetState() + Eigen::Vector2d(delta_t_ * u, 0);
    }

    
    velocity_kf_.SetState(X_prior_);
    velocity_kf_.Predict();

}

void VelocityEstimator::Update(IMUData &imu_data, WheelData &wheel_data)
{
    /* Update Step */
    is_at_least_one_wheel_data_valid_ = wheel_slip_detector_.Run(wheel_data, 
                                                                 ax_, wz_, beta_, 
                                                                 X_prior_(0), vx_best_);
                                    
    if(is_at_least_one_wheel_data_valid_)            /* If at least one wheel data is valid, we use this wheel data to update the states */
    {
        Eigen::VectorXd Z = Eigen::VectorXd(1);
        Z << vx_best_;
        velocity_kf_.UpdateLinear(Z);
    }

    wheel_speed_CG_ = wheel_slip_detector_.wheel_speed_CG_;
    is_wheel_data_valid_ = wheel_slip_detector_.is_wheel_data_valid_;                   
    wheel_rot_rate_change_ = wheel_slip_detector_.wheel_rot_rate_change_;
    velocity_deviation_ = wheel_slip_detector_.velocity_deviation_;
}

void VelocityEstimator::ReadConfig()
{
    YAML::Node config = YAML::LoadFile("src/sensorfusion/config/velocity_estimator_config.yaml");

    std::vector<double> P0_vec = config["VelocityEstimator"]["P0"].as<std::vector<double>>();
    for(Eigen::Index row = 0; row < P0_.rows(); ++row)
    {
        for(Eigen::Index col = 0; col < P0_.cols(); ++col)
        {
            P0_(row, col) = P0_vec[row * P0_.cols() + col];
        }
    }

    std::vector<double> Q_vec = config["VelocityEstimator"]["Q"].as<std::vector<double>>();
    for(Eigen::Index row = 0; row < Q_.rows(); ++row)
    {
        for(Eigen::Index col = 0; col < Q_.cols(); ++col)
        {
            Q_(row, col) = Q_vec[row * Q_.cols() + col];
        }
    }

    R_ << config["VelocityEstimator"]["R"].as<double>();

    
}

void VelocityEstimator::Init()
{
    ReadConfig();

    velocity_kf_.SetTransitionMatrix(A_);
    velocity_kf_.SetSystemNoiseCovariance(Q_);

    velocity_kf_.SetState(Eigen::Vector2d::Zero());
    velocity_kf_.SetStateCovariance(P0_);

    H_ << 1, 0;
    velocity_kf_.SetMeasurementMatrix(H_);
    velocity_kf_.SetMeasurementNoiseCovariance(R_);
}

double VelocityEstimator::GetLongiVelocity()
{
    return velocity_kf_.GetState()(0);
}

double VelocityEstimator::GetSlope()
{
    // return std::asin(velocity_kf_.getState()(1));
    return velocity_kf_.GetState()(1);
}

void VelocityEstimator::SetInitVelocity(double init_velocity)
{
    init_velocity_ = init_velocity;
}

void VelocityEstimator::SubscribeIMUData(std::deque<IMUData> &origin_imu_data_buff, 
                                         std::deque<IMUData> &subscribed_imu_data_buff)
{
    if (origin_imu_data_buff.size() > 0) {
        subscribed_imu_data_buff.insert(subscribed_imu_data_buff.end(), origin_imu_data_buff.begin(), origin_imu_data_buff.end());
    }
}

void VelocityEstimator::SubscribeWheelData(std::deque<WheelRotRateData> &origin_wheel_rot_rate_data_buff, 
                                           std::deque<WheelSteerAngleData> &origin_wheel_steer_angle_data_buff, 
                                           std::deque<WheelData> &subscribed_wheel_data_buff)
{
    if(origin_wheel_steer_angle_data_buff.size() > 0)
    {
        synced_wheel_steer_angle_data_ = origin_wheel_steer_angle_data_buff.back();
    }

    if (origin_wheel_rot_rate_data_buff.size() > 0) {
        
        WheelRotRateData new_wheel_rot_rate_data = origin_wheel_rot_rate_data_buff.back();

        if(std::fabs(new_wheel_rot_rate_data.timestamp_ - synced_wheel_steer_angle_data_.timestamp_) < 0.1)
        {
            WheelData new_wheel_data(new_wheel_rot_rate_data.wheel_rot_rate_,
                                     synced_wheel_steer_angle_data_.wheel_steer_angle_,
                                     new_wheel_rot_rate_data.timestamp_);
            subscribed_wheel_data_buff.push_back(new_wheel_data);
        }
    }
}

bool VelocityEstimator::HasIMUData() {
    if (!imu_data_buff_.empty()) 
    {
        current_imu_data_ = imu_data_buff_.front();
        imu_data_buff_.pop_front();

        return true;
    }

    return false;
}


bool VelocityEstimator::HasWheelData()
{
    if(!wheel_data_buff_.empty())
    {
        current_wheel_data_ = wheel_data_buff_.front();
        return true;
    }

    return false;
}
