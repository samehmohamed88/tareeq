#pragma once

#include "platform/io/Status.hpp"
#include "platform/sensors/odometry/OdometryData.hpp"

#include <sl/Camera.hpp>

namespace platform::zed {

class ZedVisualOdometrySensor
{
public:
    ZedVisualOdometrySensor() {

    }
private:
    PositionalTrackingParameters positionalTrackingParameters_;
    positional_tracking_param.enable_imu_fusion = true;
    positional_tracking_param.mode = sl::POSITIONAL_TRACKING_MODE::GEN_1;

    // Set parameters for Positional Tracking
    PositionalTrackingParameters positional_tracking_param;
    positional_tracking_param.enable_imu_fusion = true;
    positional_tracking_param.mode = sl::POSITIONAL_TRACKING_MODE::GEN_1;
    // positional_tracking_param.enable_area_memory = true;
    // enable Positional Tracking
    returned_state = zed.enablePositionalTracking(positional_tracking_param);
    if (returned_state != ERROR_CODE::SUCCESS) {
        print("Enabling positional tracking failed: ", returned_state);
        zed.close();
        return EXIT_FAILURE;
    }

    Pose camera_path;
};

} // namespace platform::zed