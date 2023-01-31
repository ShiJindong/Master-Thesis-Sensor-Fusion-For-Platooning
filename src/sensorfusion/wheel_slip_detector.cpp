#include "headers/wheel_slip_detector.h"

WheelSlipDetector::WheelSlipDetector()
{
    is_initialized_ = false;

    half_track_ = Eigen::Vector4d::Zero();
    distance_axle_to_CG_ = Eigen::Vector4d::Zero();
    wheel_radius_ = Eigen::Vector4d::Zero();

    valid_wheel_count_ = 0;

    wheel_speed_CG_ = Eigen::Vector4d::Zero();
    is_wheel_data_valid_ = Eigen::Vector4i::Zero();
    wheel_rot_rate_change_ = Eigen::Vector4d::Zero();
    velocity_deviation_ = Eigen::Vector4d::Zero();

    vx_best_ = 0;

    ax_ = 0.0;
    wz_ = 0.0; 
    beta_ = 0.0;
    vx_pred_ = 0.0;
}

bool WheelSlipDetector::Run(const WheelData &wheel_data_present, 
                            const double ax, const double wz, const double beta, 
                            const double vx_pred, double &vx_best)
{
    if(!is_initialized_)
    {
        ReadConfig();
        // wheel_data_his_.clear();
        is_initialized_ = true;
    }

    wheel_data_present_ = wheel_data_present;
    ax_ = ax;
    wz_ = wz;
    beta_ = beta;
    vx_pred_ = vx_pred;

    is_wheel_data_valid_ = Eigen::Vector4i(1,1,1,1);    // Set all four flags as 1 at beginning
    TransformWheelSpeed();                              // Transform the four wheel speed [m/s] to the vehicle center of gravity    
    DetectWheelSlip();                                  // Detect wheel slip of four wheels
    vx_best = SelectBestWheelSpeed();                   // vx_best: the best wheel speed of four wheels

    vx_best_ = vx_best;

    // Push the latest wheel data into wheel_data_his_
    wheel_data_his_.push_back(wheel_data_present_);
    if(static_cast<int>(wheel_data_his_.size()) > wheel_params_.step_size)
    {
        wheel_data_his_.pop_front();
    }

    // Check, if at least one wheel does not have wheel slip
    if(valid_wheel_count_ == 0)
    {
        return false;                   // return false, if all four wheels have wheel slip
    }
    else
    {
        return true;                    // return true, if at least one wheel does not have wheel slip
    }

}

void WheelSlipDetector::ReadConfig()
{
    YAML::Node config_1 = YAML::LoadFile("src/sensorfusion/config/vehicle_parameters_config.yaml");

    wheel_params_.wheel_radius_front = config_1["VehicleParamters"]["wheel_radius_front"].as<double>();
    wheel_params_.wheel_radius_rear = config_1["VehicleParamters"]["wheel_radius_rear"].as<double>();

    wheel_params_.half_track_front = config_1["VehicleParamters"]["half_track_front"].as<double>();
    wheel_params_.half_track_rear = config_1["VehicleParamters"]["half_track_rear"].as<double>();

    wheel_params_.lf = config_1["VehicleParamters"]["lf"].as<double>();
    wheel_params_.lr = config_1["VehicleParamters"]["lr"].as<double>();

    half_track_(0) =  wheel_params_.half_track_front;
    half_track_(1) =  wheel_params_.half_track_front * (-1);
    half_track_(2) =  wheel_params_.half_track_rear;
    half_track_(3) =  wheel_params_.half_track_rear * (-1);

    distance_axle_to_CG_(0) = wheel_params_.lf;
    distance_axle_to_CG_(1) = wheel_params_.lf;
    distance_axle_to_CG_(2) = wheel_params_.lr * (-1);
    distance_axle_to_CG_(3) = wheel_params_.lr * (-1);

    wheel_radius_(0) = wheel_params_.wheel_radius_front;
    wheel_radius_(1) = wheel_params_.wheel_radius_front;
    wheel_radius_(2) = wheel_params_.wheel_radius_rear;
    wheel_radius_(3) = wheel_params_.wheel_radius_rear;


    YAML::Node config_2 = YAML::LoadFile("src/sensorfusion/config/velocity_estimator_config.yaml");

    wheel_params_.step_size = config_2["VelocityEstimator"]["step_size"].as<int>();
    wheel_params_.wheel_rot_rate_change_threshold = config_2["VelocityEstimator"]["wheel_rot_rate_change_threshold"].as<double>();
    wheel_params_.velocity_deviation_threshold = config_2["VelocityEstimator"]["velocity_deviation_threshold"].as<double>();

}

void WheelSlipDetector::TransformWheelSpeed()
{
    wheel_speed_CG_ = Eigen::Vector4d::Zero();
    for(int i = 0; i < 4; ++i)
    {
        double steer_angle = wheel_data_present_.wheel_steer_angle_(i);
        double sin_steer_angle = std::sin(steer_angle);
        double cos_steer_angle = std::cos(steer_angle);
        double cos_tan_sin = cos_steer_angle + std::tan(beta_) * sin_steer_angle;
        
        if(std::fabs(cos_tan_sin) > 1e-2)
        {
            wheel_speed_CG_(i) = (wheel_data_present_.wheel_rot_rate_(i) * wheel_radius_(i) + 
                                 (half_track_(i) * cos_steer_angle - distance_axle_to_CG_(i) * sin_steer_angle) * wz_) / cos_tan_sin;
        }
        else
        {
            is_wheel_data_valid_(i) = 0;
        }
    }

}

void WheelSlipDetector::DetectWheelSlip()
{
    // Wheel Rotation Rate Change Criterion
    if(wheel_data_his_.empty())     // If there are no wheel data in wheel_data_his_, we treat all four wheel data as invalid
    {
        is_wheel_data_valid_ = Eigen::Vector4i::Zero();
        return;
    }

    for(int i = 0; i < 4; ++i)
    {
        double wheel_rot_rate_mean = 0.0;
        for(int j = 0; j < static_cast<int>(wheel_data_his_.size()); ++j)
        {
            wheel_rot_rate_mean += wheel_data_his_[j].wheel_rot_rate_(i);
        }
        wheel_rot_rate_mean /= wheel_data_his_.size();

        wheel_rot_rate_change_(i) = std::fabs(wheel_data_present_.wheel_rot_rate_(i) - wheel_rot_rate_mean);
        if(std::fabs(wheel_data_present_.wheel_rot_rate_(i) - wheel_rot_rate_mean) > wheel_params_.wheel_rot_rate_change_threshold)
        {
            /*
             * The present wheel data  of wheel [i = 0 ~ 3 for FL, FR, RL, RR] is invalid, if the present wheel rotation rate [rad/s] of this wheel, 
             * relative to the average value of the historical wheel rotation rate of this wheel, has a too rapid change.
             */
            is_wheel_data_valid_(i) = 0;
        }
    }

    // Pre-estimation Deviation Criterion

    for(int i = 0; i < 4; ++i)
    {   
        double steer_angle = wheel_data_present_.wheel_steer_angle_(i);
        double sin_steer_angle = std::sin(steer_angle);
        double cos_steer_angle = std::cos(steer_angle);
        wheel_speed_pred_(i) = (vx_pred_ - half_track_(i) * wz_) * cos_steer_angle 
                               + (vx_pred_ * std::tan(beta_) + distance_axle_to_CG_(i) * wz_) * sin_steer_angle;

        velocity_deviation_(i) = wheel_data_present_.wheel_rot_rate_(i) * wheel_radius_(i) - wheel_speed_pred_(i);

        if(std::fabs(velocity_deviation_(i)) > wheel_params_.velocity_deviation_threshold)
        {
            /* 
             * The present wheel data of wheel [i = 0 ~ 3 for FL, FR, RL, RR] is invalid,
             * if the present wheel speed [m/s] of this wheel, has a significant deviation from the wheel velocity pre-estimation
             */
            is_wheel_data_valid_(i) = 0;
        }
    }
}

double WheelSlipDetector::SelectBestWheelSpeed()
{
    double wheel_speed_max = -1e6;
    double wheel_speed_min = 1e6;
    double wheel_speed_mean = 0.0;
    valid_wheel_count_ = 0;

    for(int i = 0; i < 4; ++i)
    {
        if(is_wheel_data_valid_(i) == 1)
        {
            if(wheel_speed_CG_(i) > wheel_speed_max)
            {
                wheel_speed_max = wheel_speed_CG_(i);
            }
            if(wheel_speed_CG_(i) < wheel_speed_min)
            {
                wheel_speed_min = wheel_speed_CG_(i);
            }

            wheel_speed_mean += wheel_speed_CG_(i);
            ++valid_wheel_count_;
        }
    }

    if(valid_wheel_count_ > 0)
    {
        wheel_speed_mean /= valid_wheel_count_;
    }
    else
    {
        return 0.0;                     // If there is no valid wheel data, return 0
    }
    

    if(ax_ > 0.1)                       // Select the minimum wheel speed in Traction
    {
        return wheel_speed_min;
    }
    else if(ax_ < -0.1)                 // Select the maximum wheel speed in Braking
    {
        return wheel_speed_max;         
    }
    else                                // Select the mean value of valid wheel speed when driving at a steady speed
    {
        return wheel_speed_mean;
    }
}
