#include "av/component/StopWatch.h"

#include <rcl/time.h>
#include <rclcpp/clock.hpp>

#include <iostream>
#include <string>

namespace av {
namespace component {

StopWatch::StopWatch(rclcpp::Clock::SharedPtr clock)
    : startTime_(0, 0, RCL_ROS_TIME)
    , clock_(clock) {

    // Start the timer at creation
    tic();
}

void StopWatch::tic()
{
    // Reset the start time point
    startTime_ = clock_->now();
}

double StopWatch::toc(std::string func_name)
{
    auto now = clock_->now();

    double elapsed_nsec = (now - startTime_).nanoseconds();
    if (!func_name.empty()) {
        std::cerr << func_name << " -> toc elapsed_sec: " << elapsed_nsec / 1e9 << std::endl <<
        std::flush;
    }
    return elapsed_nsec / 1e9;  // Returns elapsed time in seconds
}

} // namespace component
} // namespace nav
