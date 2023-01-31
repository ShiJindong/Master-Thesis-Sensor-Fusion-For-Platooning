#include "headers/kalman_filter.h"
#include "../User.h"

void KalmanFilter::Predict()
{
    P_ = F_ * P_ * F_ + Q_;
}

void KalmanFilter::UpdateLinear(const Eigen::VectorXd &Z)
{
    Eigen::MatrixXd K;
    K = CalculateKalmanGain();
    
    X_ = X_ + K * (Z - H_ * X_);
    // std::cout << "X update = " << X_ << std::endl;
    
    Eigen::Index state_size = X_.size();
    Eigen::MatrixXd I = Eigen::MatrixXd::Identity(state_size, state_size);
    P_ = (I - K * H_) * P_;

}

void KalmanFilter::UpdateNonlinear(const Eigen::VectorXd &Z, const Eigen::VectorXd &Z_virtual)
{
    Eigen::MatrixXd K;
    K = CalculateKalmanGain();

    // std::cout << "(update nonlinear) K = \n" << K << std::endl;         
    
    X_ = X_ + K * (Z - Z_virtual);
    // std::cout << "X update = " << X_ << std::endl;
    
    Eigen::Index state_size = X_.size();
    Eigen::MatrixXd I = Eigen::MatrixXd::Identity(state_size, state_size);
    P_ = (I - K * H_) * P_;

}

Eigen::MatrixXd KalmanFilter::CalculateKalmanGain()
{
    Eigen::MatrixXd PH_t = P_ * H_.transpose();
    Eigen::MatrixXd K = PH_t * (H_ * PH_t + R_).inverse();

    return K;
}


void KalmanFilter::SetState(const Eigen::VectorXd &X) {X_ = X;}
void KalmanFilter::SetStateCovariance(const Eigen::MatrixXd &P) {P_ = P;}
void KalmanFilter::SetTransitionMatrix(const Eigen::MatrixXd &F) {F_ = F;}
void KalmanFilter::SetSystemNoiseCovariance(const Eigen::MatrixXd &Q) {Q_ = Q;}

void KalmanFilter::SetMeasurementMatrix(const Eigen::MatrixXd &H) {H_ = H;}
void KalmanFilter::SetMeasurementNoiseCovariance(const Eigen::MatrixXd &R) {R_ = R;}

Eigen::VectorXd KalmanFilter::GetState() {return X_;}
Eigen::MatrixXd KalmanFilter::GetStateCovariance() {return P_;}