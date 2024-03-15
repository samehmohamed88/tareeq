/*************************************************************************
    > File Name: main.cpp
    > Author: TAI Lei
    > Mail: ltai@ust.hk
    > Created Time: Thu Mar  7 19:39:14 2019
 ************************************************************************/

#include <Eigen/Eigen>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>

#include <cmath>
#include <iostream>
#include <numbers>
#include <random>
#include <tuple>

static constexpr double SIM_TIME = 50.0;
static constexpr double DT = 0.1;

auto make_2x2_diagonal(const double x0, const double x1) -> Eigen::Matrix2d
{
    // Using Eigen to define a 2x2 matrix
    Eigen::Matrix2d matrix2D;

    Eigen::Vector2d vector2D;
    vector2D << x0, x1;

    // Squaring the diagonal elements for the covariance matrix
    matrix2D = vector2D.array().square().matrix().asDiagonal();
    return matrix2D;
}

auto make_Q() -> Eigen::Matrix4d
{
    // Using Eigen to define the matrix
    Eigen::Matrix4d Q;

    // Diagonal elements representing variances
    Eigen::Vector4d variances;
    variances << 0.1,                   // variance of location on x-axis
        0.1,                            // variance of location on y-axis
        1.0 * std::numbers::pi / 180.0, // variance of yaw angle, converting degrees to radians
        1.0;                            // variance of velocity

    // Squaring the variances for the covariance matrix
    Q = variances.array().square().matrix().asDiagonal();

    // Print the matrix, if needed
    std::cout << "Covariance matrix Q:\n" << Q << std::endl;
    return Q;
}

auto make_R() -> Eigen::Matrix2d
{
    // Diagonal elements representing the observation variances
    Eigen::Matrix2d R = make_2x2_diagonal(1.0, 1.0);

    // Print the matrix, if needed
    std::cout << "Observation covariance matrix R:\n" << R << std::endl;
    return R;
}

auto make_input_noise() -> Eigen::Matrix2d
{
    double linear_noise = 1.0;                              // Linear noise
    double angular_noise = 30.0 * std::numbers::pi / 180.0; // Convert degrees to radians for angular noise

    return make_2x2_diagonal(linear_noise, angular_noise);
}

auto make_gpu_noise() -> Eigen::Matrix2d
{
    return make_2x2_diagonal(0.5, 0.5);
}

auto make_jacob_H() -> Eigen::Matrix<double, 2, 4>
{
    Eigen::Matrix<double, 2, 4> jH_;
    jH_ << 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0;
    return jH_;
}

/// @brief Jacobian of Motion Model
///
///    motion model is
///    x_{t+1} = x_t+v*dt*cos(yaw)
///    y_{t+1} = y_t+v*dt*sin(yaw)
///    yaw_{t+1} = yaw_t+omega*dt
///    v_{t+1} = v{t}
///    so
///    dx/dyaw = -v*dt*sin(yaw)
///    dx/dv = dt*cos(yaw)
///    dy/dyaw = v*dt*cos(yaw)
///    dy/dv = dt*sin(yaw)
auto jacob_F(Eigen::Vector4d x, Eigen::Vector2d u) -> Eigen::Matrix4d
{
    double yaw = x(2);
    double v = u(0);

    Eigen::Matrix4d jF_;
    jF_(0, 2) = -DT * v * std::sin(yaw);
    jF_(0, 3) = DT * std::cos(yaw);
    jF_(1, 2) = DT * v * std::cos(yaw);
    jF_(1, 3) = DT * std::sin(yaw);
    return jF_;
}

Eigen::Vector2d calc_input()
{
    double v = 1.0;       // [m/s]
    double yawrate = 0.1; // [rad/s]

    Eigen::Vector2d u;
    u << v, yawrate;

    return u;
}

// x_{t+1} = F @ x_{t} + B @ u_{t}
auto motion_model(Eigen::Matrix<double, 4, 1>& x, const Eigen::Matrix<double, 2, 1>& u)
{
    Eigen::Matrix4d F_;
    F_ << 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0;

    Eigen::Matrix<double, 4, 2> B_;
    B_ << DT * std::cos(x(2, 0)), 0.0, DT * std::sin(x(2, 0)), 0.0, 0.0, DT, 1.0, 0.0;

    x = F_ * x + B_ * u;
}

auto observation_model(const Eigen::Matrix<double, 4, 1>& x) -> Eigen::Matrix<double, 2, 1>
{
    Eigen::Matrix<double, 2, 4> H_;
    H_ << 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0;
    return H_ * x;
}

Eigen::Matrix<double, 2, 1> randomNormalVector()
{
    std::random_device rd;                      // Obtain a random number from hardware
    std::mt19937 gen(rd());                     // Seed the generator
    std::normal_distribution<> distr(0.0, 1.0); // Mean 0.0, standard deviation 1.0

    Eigen::Matrix<double, 2, 1> result;
    for (int i = 0; i < result.size(); ++i) {
        result(i) = distr(gen); // Fill each element with a normally distributed random number
    }

    return result;
}

auto observation(Eigen::Matrix<double, 4, 1>& xTrue,
                 Eigen::Matrix<double, 4, 1>& xDeadReckoning,
                 const Eigen::Matrix<double, 2, 1>& u)
    -> std::tuple<Eigen::Matrix<double, 2, 1>, Eigen::Matrix<double, 2, 1>>
{
    auto static const GPU_NOISE = make_gpu_noise();
    auto static const INPUT_NOISE = make_input_noise();

    // update xTrue by applying the motion model
    motion_model(xTrue, u);

    Eigen::Matrix<double, 2, 1> z = observation_model(xTrue) + GPU_NOISE * randomNormalVector();

    // add noise to input
    Eigen::Matrix<double, 2, 1> ud = u + INPUT_NOISE * randomNormalVector();

    motion_model(xDeadReckoning, ud);

    return {z, ud};
}

void ekf_estimation(Eigen::Matrix<double, 4, 1>& xEst,
                    Eigen::Matrix<double, 4, 4>& PEst,
                    const Eigen::Matrix<double, 2, 1>& z,
                    const Eigen::Matrix<double, 2, 1>& u)
{
    auto static const Q = make_Q();
    auto static const R = make_R();
    auto static const jH = make_jacob_H();

    auto xPred = xEst;
    motion_model(xPred, u);

    auto jF = jacob_F(xEst, u);

    auto PPred = jF * PEst * jF.transpose() + Q;

    auto zPred = observation_model(xEst);

    auto y = z - zPred;
    auto S = jH * PPred * jH.transpose() + R;
    auto K = PPred * jH.transpose() * S.inverse();

    xEst = xPred + K * y;
    PEst = (Eigen::Matrix4d::Identity() - K * jH) * PPred;
}

int main()
{
    float time = 0.0;

    Eigen::Matrix<double, 4, 1> xEst = Eigen::Matrix<double, 4, 1>::Zero();
    Eigen::Matrix<double, 4, 1> xTrue = Eigen::Matrix<double, 4, 1>::Zero();
    Eigen::Matrix4d PEst = Eigen::Matrix4d::Identity();
    Eigen::Matrix<double, 4, 1> xDeadReckoning = Eigen::Matrix<double, 4, 1>::Zero();

    while (SIM_TIME >= time) {
        time += DT;

        // control input
        auto const u = calc_input();

        // get observation and apply noise to input
        auto [z, ud] = observation(xTrue, xDeadReckoning, u);

        // apply ekf to update x and p estimates
        ekf_estimation(xEst, PEst, z, ud);

        std::cout << "updated position " << xEst << std::endl;
    }
}
