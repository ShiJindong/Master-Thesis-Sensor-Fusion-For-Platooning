#pragma once
#include "Eigen/Dense"
#include <iostream>
#include "tools.h"

class KalmanFilter
{
private:
    /* state vector */
    Eigen::VectorXd X_; 

    /* state covariance matrix */  
    Eigen::MatrixXd P_;  

    /* lineare state transition matrix or linearized jacobian of state transition matrix */               
    Eigen::MatrixXd F_;  

    /* lineare measurement matrix or linearized jacobian of measurement matrix */              
    Eigen::MatrixXd H_;

    /* covariance matrix of system noise */
    Eigen::MatrixXd Q_;

    /* covariance matrix of measurement noise */
    Eigen::MatrixXd R_;

public:
    KalmanFilter() = default;
    ~KalmanFilter() = default;

    void Predict();
    void UpdateLinear(const Eigen::VectorXd &Z);
    void UpdateNonlinear(const Eigen::VectorXd &Z, const Eigen::VectorXd &Z_virtual);                
    Eigen::MatrixXd CalculateKalmanGain();

    void SetState(const Eigen::VectorXd &X);
    void SetStateCovariance(const Eigen::MatrixXd &P);
    void SetTransitionMatrix(const Eigen::MatrixXd &F);
    void SetSystemNoiseCovariance(const Eigen::MatrixXd &Q);
    
    void SetMeasurementMatrix(const Eigen::MatrixXd &H);
    void SetMeasurementNoiseCovariance(const Eigen::MatrixXd &R);

    Eigen::VectorXd GetState();
    Eigen::MatrixXd GetStateCovariance();
};