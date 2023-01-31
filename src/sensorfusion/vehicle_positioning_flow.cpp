#include "headers/vehicle_positioning_flow.h"
#include "../rsds-client-camera.h"
#include "../User.h"

VehiclePositioningFlow::VehiclePositioningFlow()
{
    estimated_follow_longi_velocity_ = 0.0;
    estimated_lead_longi_velocity_ = 0.0;
    estimated_follow_sideslip_angle_ = 0.0;
    estimated_lead_sideslip_angle_ = 0.0;

    lidar_rel_pose_ = Eigen::Vector3d::Zero();
    camera_rel_pose_ = Eigen::Vector3d::Zero();
    fusion_rel_pose_ = Eigen::Vector3d::Zero();

    estimated_follow_abs_pose_ = Eigen::Vector3d::Zero();

    estimated_lead_states_ = Eigen::Vector4d::Zero();

    is_lidar_rel_pose_valid_ = 0;
    is_camera_rel_pose_valid_ = 0;
    num_detected_markers_ = 0;

    last_camera_timestamp_ = 0.0;
    last_valid_lidar_data_timestamp_ = 0.0;

    states_stdd_ = Eigen::VectorXd::Zero(7);

    is_initialized_ = false;

    RSDS_Init();

    follow_wheel_speed_CG_ = Eigen::Vector4d::Zero();
    is_follow_wheel_data_valid_ = Eigen::Vector4i::Zero();
    follow_wheel_rot_rate_change_ = Eigen::Vector4d::Zero();
    follow_velocity_deviation_ = Eigen::Vector4d::Zero();
}

void VehiclePositioningFlow::Run(std::shared_ptr<DataBuff> data_buff_ptr)
{
    // 1. Initialization
    // "is_initialized_" will be set to false at every simulation beginning (see function "int User_Calc (double dt)" in "User.cpp")
    if(!is_initialized_)
    {
        std::cout << "\n------------------------------------------------------------------\nInitialization starts ........" << std::endl;

        // The flags for initialization of every estimator will also be set to false
        Init();
    }


    // 2. Follow Bus State Estimation
    follow_sideslip_angle_estimator_.Run(data_buff_ptr->follow_wheel_steer_angle_data_buff_,
                                         data_buff_ptr->follow_imu_data_buff_,
                                         estimated_follow_longi_velocity_);
    estimated_follow_sideslip_angle_ = follow_sideslip_angle_estimator_.GetSideslipAngle();

    follow_velocity_estimator_.Run(data_buff_ptr->follow_imu_data_buff_,
                                   data_buff_ptr->follow_wheel_rot_rate_data_buff_,
                                   data_buff_ptr->follow_wheel_steer_angle_data_buff_,
                                   estimated_follow_sideslip_angle_,
                                   follow_data_buff_);
    estimated_follow_longi_velocity_ = follow_velocity_estimator_.GetLongiVelocity();

    // Only for plotten in IPGControl
    follow_wheel_speed_CG_ = follow_velocity_estimator_.wheel_speed_CG_;
    is_follow_wheel_data_valid_ = follow_velocity_estimator_.is_wheel_data_valid_;
    follow_wheel_rot_rate_change_ = follow_velocity_estimator_.wheel_rot_rate_change_;
    follow_velocity_deviation_ = follow_velocity_estimator_.velocity_deviation_;

#if ShowTimeLine
    // E := Timestamp of Ego Vehicle (Follow Bus) Data
    std::cout << "[E]: " << follow_data_buff_.back().timestamp_ << std::endl;
#endif


    // 3. Lead Bus State Estimation (only if the V2V communication is available)
    if((UseV2V == 1) && (!data_buff_ptr->lead_imu_data_buff_.empty()))
    {
        lead_sideslip_angle_estimator_.Run(data_buff_ptr->lead_wheel_steer_angle_data_buff_,
                                           data_buff_ptr->lead_imu_data_buff_,
                                           estimated_lead_longi_velocity_);
        estimated_lead_sideslip_angle_ = lead_sideslip_angle_estimator_.GetSideslipAngle();

        lead_velocity_estimator_.SetInitVelocity(rel_pose_estimator_.GetLeadStates()(0));
        lead_velocity_estimator_.Run(data_buff_ptr->lead_imu_data_buff_,
                                     data_buff_ptr->lead_wheel_rot_rate_data_buff_,
                                     data_buff_ptr->lead_wheel_steer_angle_data_buff_,
                                     estimated_lead_sideslip_angle_,
                                     lead_data_buff_);
        estimated_lead_longi_velocity_ = lead_velocity_estimator_.GetLongiVelocity();

#if ShowTimeLine
        // T := Timestamp of Target Vehicle (Lead Bus) Data
        std::cout << "[T]: " << lead_data_buff_.back().timestamp_
                  << "\t      --->  Valid Vehicle Data from Lead Bus received" << std::endl;
#endif
    }


    // 4. Get Relative Pose from Lidar Tracker
    if(!data_buff_ptr->scan_point_data_buff_.empty())
    {
        ScanPointData scan_point_data = data_buff_ptr->scan_point_data_buff_.back();

        // If Lidar Tracker can not detect the lead vehicle in 2 seconds, we set "last_measurement" of Lidar Tracker as "fusion_rel_pose_"
        // To tell Lidar Tracker, in which research region he could find the lead vehicle.
        if(scan_point_data.timestamp_ - last_valid_lidar_data_timestamp_ > 2.0)
        {
            lidar_tracker_.SetLastMeasurement(fusion_rel_pose_);
        }

        lidar_tracker_.processLidar(LidarL_, LidarR_, scan_point_data);

        lidar_rel_pose_ = Eigen::Vector3d(lidar_tracker_.last_output.x,
                                          lidar_tracker_.last_output.y,
                                          lidar_tracker_.last_output.phi);


        if(lidar_tracker_.valid_bus_position_ && !lidar_tracker_.no_buscluster_found_)
        {
            LidarRelPose lidar_rel_pose;
            lidar_rel_pose.rel_pose_.dx = lidar_rel_pose_(0);
            lidar_rel_pose.rel_pose_.dy = lidar_rel_pose_(1);
            lidar_rel_pose.rel_pose_.dphi = lidar_rel_pose_(2);

            lidar_rel_pose.timestamp_ = scan_point_data.timestamp_;

            is_lidar_rel_pose_valid_ = 1;
            last_valid_lidar_data_timestamp_ = scan_point_data.timestamp_;

            if(UseLidar == 1)
            {
                lidar_rel_pose_data_buff_.push_back(lidar_rel_pose);

#if ShowTimeLine
            // L := Timestamp of Lidar Scan Point Data
            std::cout << "[L]: " << scan_point_data.timestamp_ 
                    << "\t      --->  Valid Relative Pose from Lidar Tracker received: [" 
                    << lidar_rel_pose_(0) << ", "
                    << lidar_rel_pose_(1) << ", "
                    << lidar_rel_pose_(2) << "]"<< std::endl;
#endif
            }
        }
        else
        {
            is_lidar_rel_pose_valid_ = 0;

            if(UseLidar == 1)
            {
#if ShowTimeLine
                std::cout << "[L]: " << scan_point_data.timestamp_ 
                          << "\t      --->  Invalid Relative Pose from Lidar Tracker !!!" << std::endl;
#endif
            }
        }
        
        
    }

    // 5. Get Relative Pose from Camera Tracker
    RSDS_Start();
    num_detected_markers_ = RSDSIF.DataAvailable;

    if((RSDSIF.Timestamp - last_camera_timestamp_) > 1e-6)
    {
        camera_rel_pose_ = Eigen::Vector3d(RSDSIF.dx,
                                           RSDSIF.dy,
                                           RSDSIF.dphi); 

        if(num_detected_markers_ >= 2)
        {
            is_camera_rel_pose_valid_ = 1;

            if(UseCamera == 1)
            {
                CameraRelPose camera_rel_pose;
                camera_rel_pose.rel_pose_.dx = camera_rel_pose_(0);
                camera_rel_pose.rel_pose_.dy = camera_rel_pose_(1);
                camera_rel_pose.rel_pose_.dphi = camera_rel_pose_(2);

                camera_rel_pose.timestamp_ = RSDSIF.Timestamp;
                camera_rel_pose.num_detected_aruco_markers = num_detected_markers_;
                camera_rel_pose_data_buff_.push_back(camera_rel_pose);

 #if ShowTimeLine
                // C := Timestamp of Camera Image Data
                std::cout << "[C]: " << RSDSIF.Timestamp 
                          << "\t      --->  Valid Relative Pose from Camera Tracker received: [" 
                          << RSDSIF.dx << ", "
                          << RSDSIF.dy << ", "
                          << RSDSIF.dphi << "]" << std::endl;
#endif               
            }

        }
        else
        {
            is_camera_rel_pose_valid_ = 0;

            if(UseCamera == 1)
            {
#if ShowTimeLine
                std::cout << "[C]: " << RSDSIF.Timestamp 
                          << "\t      --->  Invalid Relative Pose from Camera Tracker !!!" << std::endl;
#endif
            }

        }

    }
    last_camera_timestamp_ = RSDSIF.Timestamp;
    

    // 6. Relative Pose Estimation (Informationfusion)
    if((UseLidar == 1) || (UseCamera == 1) || (UseV2V == 1))
    {
        rel_pose_estimator_.Run(follow_data_buff_, 
                                lead_data_buff_, 
                                lidar_rel_pose_data_buff_, 
                                camera_rel_pose_data_buff_);

        if(rel_pose_estimator_.is_initialized_)
        {
            fusion_rel_pose_ = rel_pose_estimator_.GetRelPose();
            rel_pose_estimator_.GetStatesStandardDeviation(states_stdd_);
        }

        estimated_follow_abs_pose_ = rel_pose_estimator_.GetAbsPoseFollow();

        estimated_lead_states_ = rel_pose_estimator_.GetLeadStates();
    }
    

    if((UseLidar != 1) && (UseCamera != 1) && (!rel_pose_estimator_.is_initialized_))
    {
        // For intialization of relativ pose estimator, Lidar or Camera must be used.
        std::cout << "[W]: " << rel_pose_estimator_.time_
                          << "\t      --->  Warning: Please start Camera-Tracker or Lidar Tracker for Initialization of Rel Pose Estimator !!!" << std::endl;
    }
    

    // 7. Clear Global and Local Data Buffer for every compute time cycle
    data_buff_ptr->ClearBuff();
    follow_data_buff_.clear();
    lead_data_buff_.clear();
    lidar_rel_pose_data_buff_.clear();
    camera_rel_pose_data_buff_.clear();
}

void VehiclePositioningFlow::Init()
{
    // Read Config File
    ReadConfig();

    // Set local variables to Zero
    estimated_follow_longi_velocity_ = 0.0;
    estimated_lead_longi_velocity_ = 0.0;
    estimated_follow_sideslip_angle_ = 0.0;
    estimated_lead_sideslip_angle_ = 0.0;

    lidar_rel_pose_ = Eigen::Vector3d::Zero();
    camera_rel_pose_ = Eigen::Vector3d::Zero();
    fusion_rel_pose_ = Eigen::Vector3d::Zero();

    estimated_follow_abs_pose_ = Eigen::Vector3d::Zero();

    estimated_lead_states_ = Eigen::Vector4d::Zero();

    is_lidar_rel_pose_valid_ = 0;
    is_camera_rel_pose_valid_ = 0;
    num_detected_markers_ = 0;
    last_camera_timestamp_ = 0.0;
    last_valid_lidar_data_timestamp_ = 0.0;

    // Set the flags for initialization of every estimator to false
    rel_pose_estimator_.is_initialized_ = false;
    follow_sideslip_angle_estimator_.is_initialized_ = false;
    lead_sideslip_angle_estimator_.is_initialized_ = false;
    follow_velocity_estimator_.is_initialized_ = false;
    lead_velocity_estimator_.is_initialized_ = false;

    // Set is_initialized_ to true
    is_initialized_ = true;
}

void VehiclePositioningFlow::ReadConfig()
{
    YAML::Node config = YAML::LoadFile("src/sensorfusion/config/run_mode_config.yaml");
    
    // The initial value of DVA will be set using value from config file
    UseLidar = config["RunMode"]["use_lidar"].as<int>();
    UseCamera = config["RunMode"]["use_camera"].as<int>();
    UseV2V = config["RunMode"]["use_V2V_communication"].as<int>();

    std::cout <<  "Use Lidar: " << (UseLidar == 1?"YES":"NO") 
              <<  "\nUse Camera: " << (UseCamera == 1?"YES":"NO") 
              <<  "\nUse V2V-Communication: " << (UseV2V == 1?"YES":"NO") << std::endl;
}





