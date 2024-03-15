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

static constexpr double SIM_TIME = 50.0;
static constexpr double DT = 0.1;
static constexpr double PI = std::numbers::pi;

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
    double linear_noise = 1.0;                 // Linear noise
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
    jH_ << 1.0, 0.0, 0.0, 0.0,
        0.0, 1.0, 0.0, 0.0;
    return jH_;
};

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
};


Eigen::Vector2d calc_input()
{
    double v = 1.0;       // [m/s]
    double yawrate = 0.1; // [rad/s]

    Eigen::Vector2d u;
    u << v, yawrate;

    return u;
}

// x_{t+1} = F @ x_{t} + B @ u_{t}
auto motion_model(const Eigen::Vector4d& x, const Eigen::Vector2d& u) -> Eigen::Vector4d
{
    Eigen::Matrix4d F_;
    F_ << 1.0, 0.0, 0.0, 0.0,
          0.0, 1.0, 0.0, 0.0,
          0.0, 0.0, 1.0, 0.0,
          0.0, 0.0, 0.0, 1.0;

    Eigen::Matrix<double, 4, 2> B_;
    B_ << DT * std::cos(x(2, 0)), 0.0,
          DT * std::sin(x(2, 0)), 0.0,
          0.0, DT,
          1.0, 0.0;

    return F_ * x + B_ * u;
};

// observation mode H
auto observation_model(const Eigen::Vector4d& x) -> Eigen::Vector2d
{
    Eigen::Matrix<double, 2, 4> H_;
    H_ << 1.0, 0.0, 0.0, 0.0,
        0.0, 1.0, 0.0, 0.0;
    return H_ * x;
};

void ekf_estimation(Eigen::Vector4d& xEst,
                    Eigen::Matrix4d& PEst,
                    const Eigen::Vector2d& z,
                    const Eigen::Vector2d& u)
{
    auto static const Q = make_Q();
    auto static const R = make_R();
    auto static const jH = make_jacob_H();

    auto xPred = motion_model(xEst, u);
    auto jF = jacob_F(xPred, u);
    
    Eigen::Vector2d PPred = jF * PEst * jF.transpose() + Q;

    auto zPred = observation_model(xPred);
    auto y = z - zPred;
    Eigen::Matrix<double, 4, 2> S = jH * PPred * jH.transpose() + R;
    Eigen::Matrix<double, 4, 2> K = PPred * jH.transpose() * S.inverse();
    xEst = xPred + K * y;
    PEst = (Eigen::Matrix4f::Identity() - K * jH) * PPred;
};


cv::Point2i cv_offset(Eigen::Vector2f e_p, int image_width = 2000, int image_height = 2000)
{
    cv::Point2i output;
    output.x = int(e_p(0) * 100) + image_width / 2;
    output.y = image_height - int(e_p(1) * 100) - image_height / 3;
    return output;
};

void ellipse_drawing(cv::Mat bg_img,
                     Eigen::Matrix2f pest,
                     Eigen::Vector2f center,
                     cv::Scalar ellipse_color = cv::Scalar(0, 0, 255))
{
    Eigen::EigenSolver<Eigen::Matrix2f> ces(pest);
    Eigen::Matrix2f e_value = ces.pseudoEigenvalueMatrix();
    Eigen::Matrix2f e_vector = ces.pseudoEigenvectors();

    double angle = std::atan2(e_vector(0, 1), e_vector(0, 0));
    cv::ellipse(bg_img,
                cv_offset(center, bg_img.cols, bg_img.rows),
                cv::Size(e_value(0, 0) * 1000, e_value(1, 1) * 1000),
                angle / PI * 180,
                0,
                360,
                ellipse_color,
                2,
                4);
};

int main()
{
    float time = 0.0;


    auto const INPUT_NOISE = make_input_noise();
    auto GPU_NOISE = make_gpu_noise();



    //  // control input
    //  Eigen::Vector2f u;
    //  u<<1.0, 0.1;
    //
    //  // nosie control input
    //  Eigen::Vector2f ud;
    //
    //  // observation z
    //  Eigen::Vector2f z;
    //
    //  // dead reckoning
    //  Eigen::Vector4f xDR;
    //  xDR<<0.0,0.0,0.0,0.0;
    //
    //  // ground truth reading
    //  Eigen::Vector4f xTrue;
    //  xTrue<<0.0,0.0,0.0,0.0;
    //
    //  // Estimation
    //  Eigen::Vector4f xEst;
    //  xEst<<0.0,0.0,0.0,0.0;
    //
    //  std::vector<Eigen::Vector4f> hxDR;
    //  std::vector<Eigen::Vector4f> hxTrue;
    //  std::vector<Eigen::Vector4f> hxEst;
    //  std::vector<Eigen::Vector2f> hz;
    //
    //  Eigen::Matrix4f PEst = Eigen::Matrix4f::Identity();
    //
    //  // Motional model covariance
    //  Eigen::Matrix4f Q = Eigen::Matrix4f::Identity();
    //  Q(0,0)=0.1 * 0.1;
    //  Q(1,1)=0.1 * 0.1;
    //  Q(2,2)=(1.0/180 * M_PI) * (1.0/180 * M_PI);
    //  Q(3,3)=0.1 * 0.1;
    //
    //  // Observation model covariance
    //  Eigen::Matrix2f  R = Eigen::Matrix2f::Identity();
    //  R(0,0)=1.0;
    //  R(1,1)=1.0;
    //
    //  // Motion model simulation error
    //  Eigen::Matrix2f Qsim = Eigen::Matrix2f::Identity();
    //  Qsim(0,0)=1.0;
    //  Qsim(1,1)=(30.0/180 * M_PI) * (30.0/180 * M_PI);
    //
    //  // Observation model simulation error
    //  Eigen::Matrix2f Rsim = Eigen::Matrix2f::Identity();
    //  Rsim(0,0)=0.5 * 0.5;
    //  Rsim(1,1)=0.5 * 0.5;
    //
    //  std::random_device rd{};
    //  std::mt19937 gen{rd()};
    //  std::normal_distribution<> gaussian_d{0,1};
    //
    //  //for visualization
    //  cv::namedWindow("ekf", cv::WINDOW_NORMAL);
    //  int count = 0;
    //
    //  while(time <= SIM_TIME){
    //    time += DT;
    //
    //    ud(0) = u(0) + gaussian_d(gen) * Qsim(0,0);
    //    ud(1) = u(1) + gaussian_d(gen) * Qsim(1,1);
    //
    //    xTrue = motion_model(xTrue, u);
    //    xDR = motion_model(xDR, ud);
    //
    //    z(0) = xTrue(0) + gaussian_d(gen) * Rsim(0,0);
    //    z(1) = xTrue(1) + gaussian_d(gen) * Rsim(1,1);
    //
    //    ekf_estimation(xEst, PEst, z, ud, Q, R);
    //
    //    hxDR.push_back(xDR);
    //    hxTrue.push_back(xTrue);
    //    hxEst.push_back(xEst);
    //    hz.push_back(z);
    //
    //    //visualization
    //    cv::Mat bg(1280,720, CV_8UC3, cv::Scalar(255,255,255));
    //    for(unsigned int j=0; j<hxDR.size(); j++){
    //
    //      // green groundtruth
    //      cv::circle(bg, cv_offset(hxTrue[j].head(2), bg.cols, bg.rows),
    //                 7, cv::Scalar(0,255,0), -1);
    //
    //      // blue estimation
    //      cv::circle(bg, cv_offset(hxEst[j].head(2), bg.cols, bg.rows),
    //                 10, cv::Scalar(255,0,0), 5);
    //
    //      // black dead reckoning
    //      cv::circle(bg, cv_offset(hxDR[j].head(2), bg.cols, bg.rows),
    //                 7, cv::Scalar(0, 0, 0), -1);
    //    }
    //
    //    // red observation
    //    for(unsigned int i=0; i<hz.size(); i++){
    //      cv::circle(bg, cv_offset(hz[i], bg.cols, bg.rows),
    //               7, cv::Scalar(0, 0, 255), -1);
    //    }
    //
    //    ellipse_drawing(bg, PEst.block(0,0,2,2), xEst.head(2));
    //
    //    cv::imshow("ekf", bg);
    //    cv::waitKey(5);
    //
    //    // std::string int_count = std::to_string(count);
    //    // cv::imwrite("./pngs/"+std::string(5-int_count.length(), '0').append(int_count)+".png", bg);
    //
    //    count++;
    //  }
}
