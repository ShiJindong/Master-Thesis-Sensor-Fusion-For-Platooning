#pragma once

#include "Eigen/Dense"
#include <cmath>

namespace tools
{
const double PI = 3.14159265358979323846;
const double bus_length = 11.89;       
const double cg_to_rear = 6.299;      		// Absolute distance [m] between bus rear and center of gravity

// Normalize angle to (-PI/2, PI/2]
inline double NormalizeAngle(double angle)
{
	if (angle > PI/2) return NormalizeAngle(angle - PI);
	if (angle <= -PI/2) return NormalizeAngle(angle + PI);
	return angle;
}

// Normalize angle to (-PI, PI]
inline double NormalizeAngle2(double angle)
{
	if (angle > PI) return NormalizeAngle2(angle - PI * 2);
	if (angle <= -PI) return NormalizeAngle2(angle + PI * 2);
	return angle;
}

// Calculate the actual relative pose between leadbus rear and followbus front (actual_rel_pose_RF)
inline Eigen::Vector3d CalculateActualRelPose(const Eigen::Vector3d &follow_bus_rear_global_pose, const Eigen::Vector3d &lead_bus_rear_global_pose)
{
	Eigen::Vector3d actual_rel_pose_RF = Eigen::Vector3d::Zero();
	double bus_length = 11.89;
	double follow_bus_front_global_yaw = follow_bus_rear_global_pose(2);

	double follow_bus_front_global_x = follow_bus_rear_global_pose(0) + bus_length * std::cos(follow_bus_front_global_yaw);
	double follow_bus_front_global_y = follow_bus_rear_global_pose(1) + bus_length * std::sin(follow_bus_front_global_yaw);


	double dx_global = lead_bus_rear_global_pose(0) - follow_bus_front_global_x;
	double dy_global = lead_bus_rear_global_pose(1) - follow_bus_front_global_y;
	

	actual_rel_pose_RF(0) = std::cos(follow_bus_front_global_yaw) * dx_global + std::sin(follow_bus_front_global_yaw) * dy_global;
	actual_rel_pose_RF(1) = - std::sin(follow_bus_front_global_yaw) * dx_global + std::cos(follow_bus_front_global_yaw) * dy_global;
	actual_rel_pose_RF(2) = lead_bus_rear_global_pose(2) - follow_bus_front_global_yaw;
	actual_rel_pose_RF(2) = NormalizeAngle2(actual_rel_pose_RF(2));

	return actual_rel_pose_RF;   
}

// We fuse the relative pose between the Center of Gravity of two buses in Kalman Filter.
// therefore we first need to transform the relative pose between leadbus rear and followbus front, to the relative pose between the Center of Gravity of two buses
inline Eigen::Vector3d TransformRF2CGs(const Eigen::Vector3d &rel_pose_RF, const double estimated_dphi)
{
    Eigen::Vector3d rel_pose_CGs = Eigen::Vector3d::Zero();
    rel_pose_CGs(0) = rel_pose_RF(0) + (bus_length - cg_to_rear) + cg_to_rear * std::cos(estimated_dphi);
    rel_pose_CGs(1) = rel_pose_RF(1) + cg_to_rear * std::sin(estimated_dphi);
    rel_pose_CGs(2) = rel_pose_RF(2);

    return rel_pose_CGs;
}

// We will plott the relative pose between leadbus rear and followbus front in IPGControl
// therefore we need to transform the relative pose between the Center of Gravity of two buses, to the relative pose between leadbus rear and followbus front
inline Eigen::Vector3d TransformCGs2RF(const Eigen::Vector3d &rel_pose_CGs)
{
    Eigen::Vector3d rel_pose_RF = Eigen::Vector3d::Zero();
	double current_dphi = rel_pose_CGs(2);
    rel_pose_RF(0) = rel_pose_CGs(0) - (bus_length - cg_to_rear) - cg_to_rear * std::cos(current_dphi);
    rel_pose_RF(1) = rel_pose_CGs(1) - cg_to_rear * std::sin(current_dphi);
    rel_pose_RF(2) = rel_pose_CGs(2);

    return rel_pose_RF;
}

// Relative Pose Estimator calculate the absolute pose of the vehicle Center of Gravity (abs_pose_CG)
// But in IPGControll we want to plott the absolute pose of the vehicle Rear (abs_pose_R)
inline Eigen::Vector3d TransformCG2R(const Eigen::Vector3d &abs_pose_CG)
{
	Eigen::Vector3d abs_pose_R = Eigen::Vector3d::Zero();
	double current_phi = abs_pose_CG(2);
	abs_pose_R(0) = abs_pose_CG(0) - cg_to_rear * std::cos(current_phi);
	abs_pose_R(1) = abs_pose_CG(1) - cg_to_rear * std::sin(current_phi);
	abs_pose_R(2) = abs_pose_CG(2);

	return abs_pose_R;
}


// Transform the relative pose of vehicle Rear (R) to the relative pose of vehicle Center of Gravity (CG)
inline Eigen::Vector3d TransformR2CG(const Eigen::Vector3d &abs_pose_R)
{
	Eigen::Vector3d abs_pose_CG = Eigen::Vector3d::Zero();
	double current_phi = abs_pose_R(2);
	abs_pose_CG(0) = abs_pose_R(0) + cg_to_rear * std::cos(current_phi);
	abs_pose_CG(1) = abs_pose_R(1) + cg_to_rear * std::sin(current_phi);
	abs_pose_CG(2) = abs_pose_R(2);

	return abs_pose_CG;
}


}