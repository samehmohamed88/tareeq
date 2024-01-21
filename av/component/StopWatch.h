#pragma once

#include <rclcpp/clock.hpp>

#include <string>

namespace nav {
namespace component {

/// @brief Stop Timer used to measure time intervals
class StopWatch
{
public:
    StopWatch(rclcpp::Clock::SharedPtr clock);
    ~StopWatch() {}

    /// Set the reference time point to the current time
    void tic();

    /// Returns the seconds elapsed from the last tic in ROS clock reference (it works also in simulation)
    double toc(std::string func_name = std::string() );

private:
    /// Reference time point
    rclcpp::Time startTime_;
    /// Node clock interface
    rclcpp::Clock::SharedPtr clock_;
};

} // namespace component
} //namespace nav

