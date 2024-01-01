#pragma once

#include "component/Component.h"
#include "rclcpp/node_options.hpp"
#include "diagnostic_updater/diagnostic_updater.hpp"

#include <sl/Camera.hpp>

#include <deque>

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
    class WindowAverage
    {
    public:
        explicit WindowAverage(size_t win_size = 15);
        ~WindowAverage();

          double setNewSize(size_t win_size);
          double addValue(double val);
          /// @brief Get the current average of the stored values
          /// @return average of the stored values
          double getAvg();

          inline size_t size() {return mVals.size();}
    private:
          size_t mWinSize;
          /// The values in the queue used to calculate the windowed average
          std::deque<double> mVals;
          /// The updated sum of the values in the queue
          double mSumVals = 0.0;

          std::mutex mQueueMux;
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

    /// Begin Diagnostic
    const float NOT_VALID_TEMP = -273.15f;
    sl::ERROR_CODE grabStatus_;
    sl::ERROR_CODE connectionStatus_;
    std::unique_ptr<WindowAverage> grabPeriodMean_sec_;
    std::unique_ptr<WindowAverage> elapsedPeriodMean_sec_;
    int systemOverloadCount_ = 0;
    float temperatureLeft_ = NOT_VALID_TEMP;
    float temperatureRight_ = NOT_VALID_TEMP;

};
} // namespace perception
} // namespace nav
