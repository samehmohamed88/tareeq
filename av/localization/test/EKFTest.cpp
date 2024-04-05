
#include "av/localization/EKF.hpp"

#include <Eigen/Dense>
#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include <iostream>
#include <memory>

using ::testing::_;
using ::testing::Invoke;
using ::testing::Return;

class EKFTest : public ::testing::Test
{
protected:
    void SetUp() override {}
};

TEST_F(EKFTest, ConstructorSuccess)
{
    auto ekf = av::localization::EKF{};

    double linearVelocity = 2.0;  // m/s
    double angularVelocity = 0.0; // rad/s
    double delta_t = 1.0;         // seconds

    // Predict step
    ekf.predict(linearVelocity, angularVelocity, delta_t);

    // Simulate IMU measurement vector
    Eigen::MatrixXd z(6, 1);
    z << 0.0, // yaw
        2.0,  // linear velocity
        0.0, // angular
        0.0, // yaw
        2.0,  // linear velocity
        0.0;  // angular velocity

    // Correct step
    ekf.correct(z);

    // Output the updated state and covariance for verification
    std::cout << "Updated State:\n" << ekf.getState() << std::endl;
    auto matrix =  ekf.getState();
//    std::cout << ">>>>>>>>>>>>> Rows: " << matrix.rows() << std::endl;
//    std::cout << ">>>>>>>>>>>>> Cols: " << matrix.cols() << std::endl;
//
//    std::cout << "Updated Covariance:\n" << ekf.getCovariance() << std::endl;

    ASSERT_TRUE(true);
}
