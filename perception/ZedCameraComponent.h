#pragma once

#include "component/Component.h"
#include "rclcpp/node_options.hpp"
#include "diagnostic_updater/diagnostic_updater.hpp"

#include <sl/Camera.hpp>

namespace nav {
namespace perception {

class ZedCameraComponent : public component::Component
{
public:
    explicit ZedCameraComponent(const rclcpp::NodeOptions& options);

    bool Init() override;
    void Run() override;
private:
    /// Initialization Functions
    void initParameters();
    void initServices();
    void initThreads();

    void getGeneralParams();
    /// End Initialization Functions

    template<typename T>
    void getParam(
        std::string paramName, T defValue, T & outVal, std::string log_info = std::string(),
        bool dynamic = false);

    void callback_updateDiagnostic(diagnostic_updater::DiagnosticStatusWrapper & stat);

private:
    /// @brief Video/Depth topic resolution
    enum class PublishResolution
    {
        /// Same camera grab resolution
        NATIVE,
        /// Custom Rescale Factor
        CUSTOM
    };

    /// Default camera model
    sl::MODEL cameraModel_ = sl::MODEL::ZED;
    int verbose_ = 1;
    /// Default camera name
    std::string cameraName_ = "zed";
    int cameraSerialNumber_ = 0;
    int cameraTimeoutSec_ = 5;
    int maxReconnectAttempts_ = 5;
    int cameraGrabFrameRate_ = 15;
    double publishFrameRate_ = 15.0;
    /// Default resolution: RESOLUTION_HD1080
    sl::RESOLUTION cameraResolution_ = sl::RESOLUTION::HD1080;
    // Use native grab resolution by default
    PublishResolution pubResolution_ = PublishResolution::NATIVE;
    /// Used to rescale data with user factor
    double customDownscaleFactor_ = 1.0;
    bool cameraSelfCalibrate_ = true;
    bool cameraFlip_ = false;

    std::string threadSchedulePolicy_;

    /// Diagnostic Updater
    diagnostic_updater::Updater diagnosticUpdater_;
};
} // namespace perception
} // namespace nav
