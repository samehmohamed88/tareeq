#pragma once

#include "av/component/Component.h"
#include "av/component/StopWatch.h"

#include <rclcpp/node_options.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <image_transport/camera_publisher.hpp>
#include <diagnostic_updater/diagnostic_updater.hpp>

#include <sl/Camera.hpp>

#include <thread>
#include <deque>

namespace av {
namespace perception {

#define TIMEZERO_ROS rclcpp::Time(0, 0, RCL_ROS_TIME)
#define TIMEZERO_SYS rclcpp::Time(0, 0, RCL_SYSTEM_TIME)

class ZedCameraComponent : public rclcpp::Node
{
public:
    explicit ZedCameraComponent(const rclcpp::NodeOptions& options);

//    bool Init() override;
//    void Run() override;

    bool Init();
    void Run();
private:
    /// Initialization Functions
//    void initParameters();
//    void initServices();
    bool startCamera();
    void initThreads();
    void initPublishers();
    void setupCameraInfoMessages(
            sl::Camera & zed,
            std::shared_ptr<sensor_msgs::msg::CameraInfo> leftCamInfoMsg,
            std::shared_ptr<sensor_msgs::msg::CameraInfo> rightCamInfoMsg, std::string leftFrameId,
            std::string rightFrameId, bool rawParam = false);

    void getGeneralParams();
    /// End Initialization Functions

    template<typename T>
    void getParam(
        std::string paramName, T defValue, T & outVal, std::string log_info = std::string(),
        bool dynamic = false);

    void callback_updateDiagnostic(diagnostic_updater::DiagnosticStatusWrapper & stat);
    // ----> Thread functions
    uint64_t frameCount_ = 0;
    void threadFunc_zedGrab();
    bool areVideoTopicsSubscribed();
    void retrieveImages();
    void publishImages(rclcpp::Time & publishTimestamp);
    /// TODO: apply video settings
    // void applyVideoSettings();

    rclcpp::Time slTime2Ros(sl::Timestamp t, rcl_clock_type_t clock_type = RCL_ROS_TIME);
    std::unique_ptr<sensor_msgs::msg::Image> imageToROSmsg(
                    sl::Mat & img, std::string frameId, rclcpp::Time t);

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

    // ZED SDK
    sl::Camera zed_;
    sl::InitParameters initParameters_;
    sl::RuntimeParameters runtimeParameters_;

    bool threadStop_ = false;

    /// Default camera model
    sl::MODEL cameraModel_ = sl::MODEL::ZED;
    int verbose_ = 1;
    /// Default camera name
    std::string cameraName_ = "zed";
    int cameraSerialNumber_ = 0;
    /// Camera FW version
    unsigned int cameraFwVersion_;
    int cameraTimeoutSec_ = 5;
    int maxReconnectAttempts_ = 5;
    int cameraGrabFrameRate_ = 15;
    double publishFrameRate_ = 15.0;
    /// Default resolution: RESOLUTION_HD1080
    sl::RESOLUTION cameraResolution_ = sl::RESOLUTION::HD1080;
    /// Use native grab resolution by default
    PublishResolution pubResolution_ = PublishResolution::NATIVE;
    /// Used to rescale image with user provided factor
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
    /// Main grab thread
    std::thread grabThread_;
    std::unique_ptr<WindowAverage> grabPeriodMean_sec_;
    std::unique_ptr<WindowAverage> elapsedPeriodMean_sec_;
    std::unique_ptr<WindowAverage> videoPeriodMean_sec_;
    std::unique_ptr<WindowAverage> videoElapsedMean_sec_;
    int systemOverloadCount_ = 0;
    float temperatureLeft_ = NOT_VALID_TEMP;
    float temperatureRight_ = NOT_VALID_TEMP;

    // ----> Stereolabs Mat Info
    /// Camera frame width
    int cameraFrameWidth_;
    /// Camera frame height
    int cameraFrameHeight_;
    sl::Resolution matrixResolution_;
    // <---- Stereolabs Mat Info

    // ----> Messages (ONLY THOSE NOT CHANGING WHILE NODE RUNS)
    typedef std::unique_ptr<sensor_msgs::msg::Image> ImageMessage;
    typedef std::shared_ptr<sensor_msgs::msg::CameraInfo> CameraInfoMessage;

    // Camera infos
    CameraInfoMessage leftCameraInfoMessage_;
    CameraInfoMessage rightCameraInfoMessage_;
    CameraInfoMessage leftCameraInfoRawMessage_;
    CameraInfoMessage rightCameraInfoRawMessage_;
    // <---- Messages

    // ----> Topics
    std::string topicRoot_ = "~/";
    std::string leftCameraFrameId_;
    std::string rightCameraFrameId_;
    // <---- Topics

    // QoS parameters
    // https://github.com/ros2/ros2/wiki/About-Quality-of-Service-Settings
    rclcpp::QoS videoQos_;

    // ----> Publishers
    image_transport::CameraPublisher leftImagePublisher_;
    image_transport::CameraPublisher leftRawImagePublisher_;
    image_transport::CameraPublisher rightImagePublisher_;
    image_transport::CameraPublisher rightRawImagePublisher_;

    component::StopWatch grabFreqTimer_;
    component::StopWatch imagePublisherFrequencyTimer_;

    rclcpp::Time frameTimestamp_;
    sl::Timestamp lastGrabTimestamp_ = 0;  // Used to calculate stable publish frequency
    size_t leftNumberSubscribed_;
    size_t leftRawNumberSubscribed_;
    size_t rightNumberSubscribed_;
    size_t rightRawNumberSubscribed_;

    // <---- Publisher variables
    sl::Timestamp mSdkGrabTS = 0;
    sl::Mat matrixLeftImage_, matrixLefImageRaw_;
    sl::Mat matrixRightImage_, matrixRightImageRaw_;

    void publishImageWithInfo(
                    sl::Mat & img, image_transport::CameraPublisher & imagePublisher, CameraInfoMessage & cameraInfoMessage,
            std::string imgFrameId, rclcpp::Time t);

};
} // namespace perception
} // namespace nav
