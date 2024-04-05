#pragma once

#include <Eigen/Dense>

#include <cmath>

namespace av::localization {

enum StateIndex
{
    coordinate_x_ = 0,
    coordinate_y_,
    theta,
    linearVelocity,
    angularVelocity
};

class EKF
{
public:
    EKF()
        : state_{Eigen::MatrixXd::Zero(5, 1)}
        , Q_{Eigen::MatrixXd::Identity(5, 5)}
        , H_{generate_H()}
        , R_{generate_R()}
        , motionModelJacobian_{Eigen::MatrixXd::Identity(5, 5)}
        , covariance_{Eigen::MatrixXd::Identity(5, 5)}
    {
        state_ << 0, 0, 1.5708, 0, 0;
    };

    void predict(double linearVelocity, double angularVelocity, double delta_t)
    {
        motionModel(linearVelocity, angularVelocity, delta_t);
        updateMotionCovariance();
    }

    /// z is the measurement vector
    void correct(const Eigen::MatrixXd& z)
    {
        // Predict the measurement
        Eigen::MatrixXd z_pred = H_ * state_;

        // Measurement residual
        Eigen::MatrixXd y = z - z_pred;

        // Calculate Kalman Gain
        Eigen::MatrixXd S = H_ * covariance_ * H_.transpose() + R_;
        Eigen::MatrixXd K = covariance_ * H_.transpose() * S.inverse();

        // Update state estimate
        state_ = state_ + K * y;

        // Update covariance matrix
        Eigen::MatrixXd I = Eigen::MatrixXd::Identity(state_.rows(), state_.rows());
        covariance_ = (I - K * H_) * covariance_;
    }

    const Eigen::MatrixXd getState() const { return state_; }

    const Eigen::MatrixXd getCovariance() const { return covariance_; }

private:
    void motionModel(double linearVelocity, double angularVelocity, double delta_t)
    {
        double sinTheta = std::sin(state_(StateIndex::theta, 0));
        double cosTheta = std::cos(state_(StateIndex::theta, 0));

        // Predict the next state
        Eigen::MatrixXd predictedState = Eigen::MatrixXd::Zero(5, 1);
        predictedState(StateIndex::coordinate_x_, 0) =
            state_(StateIndex::coordinate_x_, 0) + linearVelocity * cosTheta * delta_t;
        predictedState(StateIndex::coordinate_y_, 0) =
            state_(StateIndex::coordinate_y_, 0) + linearVelocity * sinTheta * delta_t;
        predictedState(StateIndex::theta, 0) = state_(StateIndex::theta, 0) + angularVelocity * delta_t;
        predictedState(StateIndex::linearVelocity, 0) = linearVelocity;
        predictedState(StateIndex::angularVelocity, 0) = angularVelocity;

        state_ = predictedState; // Update state with predicted value

        // Update Jacobian matrix
        motionModelJacobian_(0, 2) = -linearVelocity * sinTheta * delta_t;
        motionModelJacobian_(0, 3) = cosTheta * delta_t;
        motionModelJacobian_(1, 2) = linearVelocity * cosTheta * delta_t;
        motionModelJacobian_(1, 3) = sinTheta * delta_t;
        motionModelJacobian_(2, 4) = delta_t;
    }

    void updateMotionCovariance()
    {
        covariance_ = motionModelJacobian_ * covariance_ * motionModelJacobian_.transpose() + Q_;
    }

    Eigen::MatrixXd generate_H()
    {
        Eigen::MatrixXd H(6, 5); // 6 measurements, 5 state variables
        H.setZero();
        H(0, StateIndex::theta) = 1;           // Mapping theta to yaw measurement of IMU1
        H(1, StateIndex::linearVelocity) = 1;  // Mapping v to linear velocity measured (integral of IMU1 acceleration)
        H(2, StateIndex::angularVelocity) = 1; // Mapping omega to gyro measurement of IMU1
        H(3, StateIndex::theta) = 1;           // Mapping theta to yaw measurement of IMU2
        H(4, StateIndex::linearVelocity) = 1;  // Mapping v to linear velocity measured (integral of IMU2 acceleration)
        H(5, StateIndex::angularVelocity) = 1; // Mapping omega to gyro measurement of IMU2

        return H;
    }

    Eigen::MatrixXd generate_R()
    {
        double sigma_theta1 = 0.005;  // Standard deviation for orientation angle of IMU1
        double sigma_v1 = 0.15;      // Standard deviation for linear velocity of IMU1
        double sigma_omega1 = 0.005; // Standard deviation for angular velocity of IMU1

        double sigma_theta2 = 0.45; // Standard deviation for orientation angle of IMU2
        double sigma_v2 = 0.55;     // Standard deviation for linear velocity of IMU2
        double sigma_omega2 = 0.45; // Standard deviation for angular velocity of IMU2

        Eigen::MatrixXd R(6, 6);
        R.setZero();

        // For IMU1
        R(0, 0) = std::pow(sigma_theta1, 2); // Variance for theta from IMU1
        R(1, 1) = std::pow(sigma_v1, 2);     // Variance for linear velocity from IMU1
        R(2, 2) = std::pow(sigma_omega1, 2); // Variance for angular velocity from IMU1

        // For IMU2
        R(3, 3) = std::pow(sigma_theta2, 2); // Variance for theta from IMU2
        R(4, 4) = std::pow(sigma_v2, 2);     // Variance for linear velocity from IMU2
        R(5, 5) = std::pow(sigma_omega2, 2); // Variance for angular velocity from IMU2

        return R;
    }

    /// X = [x, y, theta, v, omega]
    Eigen::MatrixXd state_;
    Eigen::MatrixXd Q_;
    Eigen::MatrixXd H_;
    Eigen::MatrixXd R_;

    Eigen::MatrixXd motionModelJacobian_;
    Eigen::MatrixXd covariance_;
};

} // namespace av::localization
