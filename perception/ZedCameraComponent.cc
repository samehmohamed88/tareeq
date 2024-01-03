#include "perception/ZedCameraComponent.h"
#include "component/StopWatch.h"
#include "component/Component.h"

#include <rclcpp/node_options.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sl/Camera.hpp>

#include <chrono>

using namespace std::chrono_literals;
//using namespace std::placeholders;

namespace nav {
namespace perception {

#ifndef DEG2RAD
#define DEG2RAD 0.017453293
#define RAD2DEG 57.295777937
#endif

#define ROS_COORDINATE_SYSTEM sl::COORDINATE_SYSTEM::RIGHT_HANDED_Z_UP_X_FWD
#define ROS_MEAS_UNITS sl::UNIT::METER

ZedCameraComponent::ZedCameraComponent(const rclcpp::NodeOptions& options)
    : component::Component("", options)
    , diagnosticUpdater_(this)
{}

bool ZedCameraComponent::Init()
{
    // Parameters initialization
    getGeneralParams();

    // ----> Diagnostic initialization
    diagnosticUpdater_.add("ZED Diagnostic", this, &ZedCameraComponent::callback_updateDiagnostic);
    std::string hw_id = std::string("Stereolabs camera: ") + cameraName_;
    diagnosticUpdater_.setHardwareID(hw_id);
    // <---- Diagnostic initialization

    // ----> Start camera
    if (!startCamera()) {
        exit(EXIT_FAILURE);
    }
    // <---- Start camera

    // Dynamic parameters callback
//    mParamChangeCallbackHandle =
//        add_on_set_parameters_callback(std::bind(&ZedCameraComponent::callback_paramChange, this, _1));
    return true;
}

bool ZedCameraComponent::startCamera()
{
    RCLCPP_INFO(get_logger(), "***** STARTING CAMERA *****");

    // ----> SDK version
    RCLCPP_INFO(
        get_logger(), "ZED SDK Version: %d.%d.%d - Build %s", ZED_SDK_MAJOR_VERSION, ZED_SDK_MINOR_VERSION,
        ZED_SDK_PATCH_VERSION, ZED_SDK_BUILD_ID);
    // <---- SDK version

    // ----> TF2 Transform
//    mTfBuffer = std::make_unique<tf2_ros::Buffer>(get_clock());
//    mTfListener =
//        std::make_unique<tf2_ros::TransformListener>(*mTfBuffer);  // Start TF Listener thread
//    mTfBroadcaster = std::make_unique<tf2_ros::TransformBroadcaster>(this);
    // <---- TF2 Transform

    // ----> ZED configuration
    RCLCPP_INFO(get_logger(), "*** CAMERA OPENING ***");
    initParameters_.camera_fps = cameraGrabFrameRate_;
    initParameters_.grab_compute_capping_fps = static_cast<float>(publishFrameRate_);
    initParameters_.camera_resolution = static_cast<sl::RESOLUTION>(cameraResolution_);

    if (cameraSerialNumber_ > 0) {
        initParameters_.input.setFromSerialNumber(cameraSerialNumber_);
    }

    initParameters_.coordinate_system = ROS_COORDINATE_SYSTEM;
    initParameters_.coordinate_units = ROS_MEAS_UNITS;
    initParameters_.sdk_verbose = verbose_;
    initParameters_.camera_image_flip = cameraFlip_;

    initParameters_.camera_disable_self_calib = !cameraSelfCalibrate_;
    initParameters_.enable_image_enhancement = true;
    initParameters_.enable_right_side_measure = false;

    initParameters_.async_grab_camera_recovery = true; // Camera recovery is handled asynchronously to provide information about this status
    // <---- ZED configuration

    // ----> Try to connect to a camera
    component::StopWatch connectTimer(get_clock());

    threadStop_ = false;

    while (1) {
        rclcpp::sleep_for(500ms);

        connectionStatus_ = zed_.open(initParameters_);

        if (connectionStatus_ == sl::ERROR_CODE::SUCCESS) {
            RCLCPP_DEBUG_STREAM(get_logger(), "Opening successful");
            break;
        }

        RCLCPP_WARN(get_logger(), "Error opening camera: %s", sl::toString(connectionStatus_).c_str());

        if (connectionStatus_ == sl::ERROR_CODE::CAMERA_DETECTION_ISSUE )
        {
            RCLCPP_INFO(get_logger(), "Please verify the camera connection");
        }

        if (!rclcpp::ok() || threadStop_) {
            RCLCPP_INFO(get_logger(), "ZED activation interrupted by user.");
            return false;
        }

        if (connectTimer.toc() > maxReconnectAttempts_ * cameraTimeoutSec_) {
            RCLCPP_ERROR(get_logger(), "Camera detection timeout");
            return false;
        }
        rclcpp::sleep_for(std::chrono::seconds(cameraTimeoutSec_));
    }
    // ----> Try to connect to a camera

    // ----> Camera information
    sl::CameraInformation cameraInfo = zed_.getCameraInformation();

    float realFps = cameraInfo.camera_configuration.fps;
    if (realFps != static_cast<float>(cameraGrabFrameRate_)) {
        RCLCPP_WARN_STREAM(
            get_logger(), "!!! `general.grab_frame_rate` value is not valid: '" <<
                              cameraGrabFrameRate_ << "'. Automatically replaced with '" << realFps <<
                              "'. Please fix the parameter !!!");
        cameraGrabFrameRate_ = realFps;
    }

    // Camera model
    cameraModel_ = cameraInfo.camera_model;
    RCLCPP_INFO_STREAM(get_logger(), " * Camera Model  -> " << sl::toString(cameraModel_).c_str());
    cameraSerialNumber_ = cameraInfo.serial_number;

    RCLCPP_INFO_STREAM(get_logger(), " * Serial Number -> " << cameraSerialNumber_);
#if (ZED_SDK_MINOR_VERSION == 0 && ZED_SDK_PATCH_VERSION >= 6)
    RCLCPP_INFO_STREAM(
        get_logger(),
        " * Focal Lenght -> " << cameraInfo.camera_configuration.calibration_parameters.left_cam.focal_length_metric <<
            " mm");
#endif

    RCLCPP_INFO_STREAM(
        get_logger(),
        " * Input\t -> " << sl::toString(zed_.getCameraInformation().input_type).c_str());

    // Firmwares
    cameraFwVersion_ = cameraInfo.camera_configuration.firmware_version;
    RCLCPP_INFO_STREAM(get_logger(), " * Camera FW Version  -> " << cameraFwVersion_);

    cameraFrameWidth_ = cameraInfo.camera_configuration.resolution.width;
    cameraFrameHeight_ = cameraInfo.camera_configuration.resolution.height;
    RCLCPP_INFO_STREAM(
        get_logger(), " * Camera grab frame size -> " << cameraFrameWidth_ << "x" << cameraFrameHeight_);

    int publishWidth, publishHeight;
    publishWidth = static_cast<int>(std::round(cameraFrameWidth_ / customDownscaleFactor_));
    publishHeight = static_cast<int>(std::round(cameraFrameHeight_ / customDownscaleFactor_));

    if (publishWidth > cameraFrameWidth_ || publishHeight > cameraFrameHeight_) {
        RCLCPP_WARN_STREAM(
            get_logger(),
            "The publishing resolution (" <<
                publishWidth << "x" << publishWidth <<
                ") cannot be higher than the grabbing resolution (" <<
                cameraFrameWidth_ << "x" << cameraFrameHeight_ <<
                "). Using grab resolution for output messages.");
        publishWidth = cameraFrameWidth_;
        publishHeight = cameraFrameHeight_;
    }

    matrixResolution_ = sl::Resolution(publishWidth, publishHeight);
    RCLCPP_INFO_STREAM(
        get_logger(), " * Publishing frame size  -> " << matrixResolution_.width << "x" << matrixResolution_.height);
    // <---- Camera information

    // ----> Camera Info messages
    rgbCameraInfoMessage_ = std::make_shared<sensor_msgs::msg::CameraInfo>();
    leftCameraInfoMessage_ = std::make_shared<sensor_msgs::msg::CameraInfo>();
    rightCameraInfoMessage_ = std::make_shared<sensor_msgs::msg::CameraInfo>();
    rgbCameraInfoRawMessage_ = std::make_shared<sensor_msgs::msg::CameraInfo>();
    leftCameraInfoRawMessage_ = std::make_shared<sensor_msgs::msg::CameraInfo>();
    rightCameraInfoRawMessage_ = std::make_shared<sensor_msgs::msg::CameraInfo>();

    setTFCoordFrameNames();  // Requires mZedRealCamModel available only after camera opening

    fillCamInfo(mZed, mLeftCamInfoMsg, mRightCamInfoMsg, mLeftCamOptFrameId, mRightCamOptFrameId);
    fillCamInfo(
        mZed, mLeftCamInfoRawMsg, mRightCamInfoRawMsg, mLeftCamOptFrameId, mRightCamOptFrameId, true);
    mRgbCamInfoMsg = mLeftCamInfoMsg;
    mRgbCamInfoRawMsg = mLeftCamInfoRawMsg;
    mDepthCamInfoMsg = mLeftCamInfoMsg;
    // <---- Camera Info messages

    initPublishers();  // Requires mZedRealCamModel available only after camera
                      // opening
    initSubscribers();

    // Disable AEC_AGC and Auto Whitebalance to trigger it if user set it to
    // automatic
    if (!mSvoMode && !mSimMode) {
        mZed.setCameraSettings(sl::VIDEO_SETTINGS::AEC_AGC, 0);
        mZed.setCameraSettings(sl::VIDEO_SETTINGS::WHITEBALANCE_AUTO, 0);
        // Force parameters with a dummy grab
        mZed.grab();
    }

    // Initialialized timestamp to avoid wrong initial data
    // ----> Timestamp
    if (mSvoMode) {
        mFrameTimestamp = sl_tools::slTime2Ros(mZed.getTimestamp(sl::TIME_REFERENCE::CURRENT));
    } else if (mSimMode) {
        if (mUseSimTime) {
            mFrameTimestamp = get_clock()->now();
        } else {
            mFrameTimestamp = sl_tools::slTime2Ros(mZed.getTimestamp(sl::TIME_REFERENCE::IMAGE));
        }
    } else {
        mFrameTimestamp = sl_tools::slTime2Ros(mZed.getTimestamp(sl::TIME_REFERENCE::IMAGE));
    }
    // <---- Timestamp

    // ----> Initialize Diagnostic statistics
    mElabPeriodMean_sec = std::make_unique<sl_tools::WinAvg>(mCamGrabFrameRate);
    mGrabPeriodMean_sec = std::make_unique<sl_tools::WinAvg>(mCamGrabFrameRate);
    mVideoDepthPeriodMean_sec = std::make_unique<sl_tools::WinAvg>(mCamGrabFrameRate);
    mVideoDepthElabMean_sec = std::make_unique<sl_tools::WinAvg>(mCamGrabFrameRate);
    mPcPeriodMean_sec = std::make_unique<sl_tools::WinAvg>(mCamGrabFrameRate);
    mPcProcMean_sec = std::make_unique<sl_tools::WinAvg>(mCamGrabFrameRate);
    mObjDetPeriodMean_sec = std::make_unique<sl_tools::WinAvg>(mCamGrabFrameRate);
    mObjDetElabMean_sec = std::make_unique<sl_tools::WinAvg>(mCamGrabFrameRate);
    mBodyTrkPeriodMean_sec = std::make_unique<sl_tools::WinAvg>(mCamGrabFrameRate);
    mBodyTrkElabMean_sec = std::make_unique<sl_tools::WinAvg>(mCamGrabFrameRate);
    mImuPeriodMean_sec = std::make_unique<sl_tools::WinAvg>(mSensPubRate);
    mBaroPeriodMean_sec = std::make_unique<sl_tools::WinAvg>(mSensPubRate);
    mMagPeriodMean_sec = std::make_unique<sl_tools::WinAvg>(mSensPubRate);
    mPubFusedCloudPeriodMean_sec = std::make_unique<sl_tools::WinAvg>(mPcPubRate);
    mPubOdomTF_sec = std::make_unique<sl_tools::WinAvg>(mSensPubRate);
    mPubPoseTF_sec = std::make_unique<sl_tools::WinAvg>(mSensPubRate);
    mPubImuTF_sec = std::make_unique<sl_tools::WinAvg>(mSensPubRate);
    mGnssFix_sec = std::make_unique<sl_tools::WinAvg>(10);
    // <---- Initialize Diagnostic statistics

    if (mGnssFusionEnabled) {
        DEBUG_GNSS("Initialize Fusion module");
        // ----> Initialize Fusion module

        // Fusion parameters
        mFusionInitParams.coordinate_system = ROS_COORDINATE_SYSTEM;
        mFusionInitParams.coordinate_units = ROS_MEAS_UNITS;
        mFusionInitParams.verbose = mVerbose != 0;
        mFusionInitParams.output_performance_metrics = true;
        mFusionInitParams.timeout_period_number = 20; // TODO(Walter) Evaluate this: mCamGrabFrameRate * mCamTimeoutSec;

        // Fusion initialization
        sl::FUSION_ERROR_CODE fus_err = mFusion.init(mFusionInitParams);
        if (fus_err != sl::FUSION_ERROR_CODE::SUCCESS) {
            RCLCPP_ERROR_STREAM(
                get_logger(), "Error initializing the Fusion module: " << sl::toString(
                                                                              fus_err).c_str() << ".");
            exit(EXIT_FAILURE);
        }
        DEBUG_GNSS(" Fusion params OK");

        mFusionConfig =
            std::make_shared<sl::FusionConfiguration>();

        if (mSimMode) {
            //TODO(Walter) Modify when support for streaming input is added in the SDK
            //mFusionConfig->input_type.setFromStream(mSimAddr, mSimPort);
            mFusionConfig->input_type.setFromSerialNumber(mCamSerialNumber);
            mFusionConfig->communication_parameters.setForSharedMemory();
        } else if (mSvoMode) {
            mFusionConfig->input_type.setFromSVOFile(mSvoFilepath.c_str());
            mFusionConfig->communication_parameters.setForSharedMemory();
        } else {
            mFusionConfig->input_type.setFromSerialNumber(mCamSerialNumber);
            mFusionConfig->communication_parameters.setForSharedMemory();
        }
        mFusionConfig->serial_number = mCamSerialNumber;
        mFusionConfig->pose = sl::Transform::identity();

        DEBUG_GNSS(" Fusion communication params OK");

        // Camera identifier
        mCamUuid.sn = mCamSerialNumber;

        // Enable camera publishing to Fusion
        mZed.startPublishing(mFusionConfig->communication_parameters);
        DEBUG_GNSS(" Camera publishing OK");

        // Fusion subscribe to camera data
        fus_err = mFusion.subscribe(
            mCamUuid, mFusionConfig->communication_parameters,
            mFusionConfig->pose);
        if (fus_err != sl::FUSION_ERROR_CODE::SUCCESS) {
            RCLCPP_ERROR_STREAM(
                get_logger(), "Error initializing the Fusion module: " << sl::toString(fus_err).c_str());
            exit(EXIT_FAILURE);
        }
        DEBUG_GNSS(" Fusion subscribing OK");
        DEBUG_GNSS("Fusion module ready");
        // <---- Initialize Fusion module
    }

    // Init and start threads
    initThreads();

    return true;
}  // namespace stereolabs


template<typename T>
void ZedCameraComponent::getParam(std::string paramName, T defValue, T& outVal, std::string log_info, bool dynamic)
{
    rcl_interfaces::msg::ParameterDescriptor descriptor;
    descriptor.read_only = !dynamic;

    declare_parameter(paramName, rclcpp::ParameterValue(defValue), descriptor);

    if (!get_parameter(paramName, outVal)) {
        RCLCPP_WARN_STREAM(get_logger(),
                           "The parameter '"
                               << paramName
                               << "' is not available or is not valid, using the default value: " << defValue);
    }

    if (!log_info.empty()) {
        RCLCPP_INFO_STREAM(get_logger(), log_info << outVal);
    }
}

void ZedCameraComponent::getGeneralParams()
{
    rclcpp::Parameter paramVal;
    std::string paramName;

    RCLCPP_INFO(get_logger(), "*** GENERAL parameters ***");

    std::string camera_model = "zed";
    getParam("general.camera_model", camera_model, camera_model);
    if (camera_model == "zed") {
        cameraModel_ = sl::MODEL::ZED;
    } else if (camera_model == "zedm") {
        cameraModel_ = sl::MODEL::ZED_M;
    } else if (camera_model == "zed2") {
        cameraModel_ = sl::MODEL::ZED2;
    } else if (camera_model == "zed2i") {
        cameraModel_ = sl::MODEL::ZED2i;
    } else if (camera_model == "zedx") {
        cameraModel_ = sl::MODEL::ZED_X;
//        if (!IS_JETSON) {
//            RCLCPP_ERROR_STREAM(get_logger(),
//                                "Camera model " << sl::toString(cameraModel_).c_str()
//                                                << " is available only with NVIDIA Jetson devices.");
//            exit(EXIT_FAILURE);
//        }
    } else if (camera_model == "zedxm") {
        cameraModel_ = sl::MODEL::ZED_XM;
//        if (!IS_JETSON) {
//            RCLCPP_ERROR_STREAM(get_logger(),
//                                "Camera model " << sl::toString(cameraModel_).c_str()
//                                                << " is available only with NVIDIA Jetson devices.");
//            exit(EXIT_FAILURE);
//        }
    } else {
        RCLCPP_ERROR_STREAM(get_logger(), "Camera model not valid in parameter values: " << camera_model);
    }
    RCLCPP_INFO_STREAM(get_logger(), " * Camera model: " << camera_model << " - " << cameraModel_);

    getParam("general.sdk_verbose", verbose_, verbose_, " * SDK Verbose: ");
    getParam("general.camera_name", cameraName_, cameraName_, " * Camera name: ");
    getParam("general.serial_number", cameraSerialNumber_, cameraSerialNumber_, " * Camera SN: ");
    getParam("general.camera_timeout_sec", cameraTimeoutSec_, cameraTimeoutSec_, " * Camera timeout [sec]: ");
    getParam(
        "general.camera_max_reconnect", maxReconnectAttempts_, maxReconnectAttempts_, " * Camera reconnection attemps before shutting down: ");
    getParam("general.grab_frame_rate", cameraGrabFrameRate_, cameraGrabFrameRate_, " * Camera framerate: ");
    
    std::string resolution = "AUTO";
    getParam("general.grab_resolution", resolution, resolution);
    if (resolution == "AUTO") {
        cameraResolution_ = sl::RESOLUTION::AUTO;
    }
//    else if (sl_tools::isZEDX(cameraModel_)) {
//        if (resolution == "HD1200") {
//            cameraResolution_ = sl::RESOLUTION::HD1200;
//        } else if (resolution == "HD1080") {
//            cameraResolution_ = sl::RESOLUTION::HD1080;
//        } else if (resolution == "SVGA") {
//            cameraResolution_ = sl::RESOLUTION::SVGA;
//        } else {
//            RCLCPP_WARN(get_logger(),
//                        "Not valid 'general.grab_resolution' value: '%s'. Using 'AUTO' setting.",
//                        resol.c_str());
//            cameraResolution_ = sl::RESOLUTION::AUTO;
//        }
//        RCLCPP_INFO_STREAM(get_logger(), " * Camera resolution: " << sl::toString(cameraResolution_).c_str());
//    }
    else {
        if (resolution == "HD2K") {
            cameraResolution_ = sl::RESOLUTION::HD2K;
        } else if (resolution == "HD1080") {
            cameraResolution_ = sl::RESOLUTION::HD1080;
        } else if (resolution == "HD720") {
            cameraResolution_ = sl::RESOLUTION::HD720;
        } else if (resolution == "VGA") {
            cameraResolution_ = sl::RESOLUTION::VGA;
        } else {
            RCLCPP_WARN(get_logger(),
                        "Not valid 'general.grab_resolution' value: '%s'. Using 'AUTO' setting.",
                        resolution.c_str());
            cameraResolution_ = sl::RESOLUTION::AUTO;
        }
        RCLCPP_INFO_STREAM(get_logger(), " * Camera resolution: " << sl::toString(cameraResolution_).c_str());
    }

    std::string publishResolution = "MEDIUM";
    getParam("general.pub_resolution", publishResolution, publishResolution);
    if (publishResolution == "NATIVE") {
        pubResolution_ = PublishResolution::NATIVE;
    } else if (publishResolution == "CUSTOM") {
        pubResolution_ = PublishResolution::CUSTOM;
    } else {
        RCLCPP_WARN(get_logger(),
                    "Not valid 'general.pub_resolution' value: '%s'. Using default setting instead.",
                    publishResolution.c_str());
        publishResolution = "NATIVE";
        pubResolution_ = PublishResolution::NATIVE;
    }
    RCLCPP_INFO_STREAM(get_logger(), " * Publishing resolution: " << publishResolution.c_str());

    if (pubResolution_ == PublishResolution::CUSTOM) {
        getParam("general.pub_downscale_factor",
                 customDownscaleFactor_,
                 customDownscaleFactor_,
                 " * Publishing downscale factor: ");
    } else {
        customDownscaleFactor_ = 1.0;
    }

//    std::string parsed_str = getParam("general.region_of_interest", mRoiParam);
//    RCLCPP_INFO_STREAM(get_logger(), " * Region of interest: " << parsed_str.c_str());

    getParam("general.self_calib", cameraSelfCalibrate_, cameraSelfCalibrate_);
    RCLCPP_INFO_STREAM(get_logger(), " * Camera self calibration: " << (cameraSelfCalibrate_ ? "TRUE" : "FALSE"));

    getParam("general.camera_flip", cameraFlip_, cameraFlip_);
    RCLCPP_INFO_STREAM(get_logger(), " * Camera flip: " << (cameraFlip_ ? "TRUE" : "FALSE"));

    // Dynamic parameters
    getParam("general.pub_frame_rate", publishFrameRate_, publishFrameRate_, "", false);
    if (publishFrameRate_ > cameraGrabFrameRate_) {
        RCLCPP_WARN(get_logger(), "'pub_frame_rate' cannot be bigger than 'grab_frame_rate'");
        publishFrameRate_ = cameraGrabFrameRate_;
    }
    if (publishFrameRate_ < 0.1) {
        RCLCPP_WARN(get_logger(), "'pub_frame_rate' cannot be lower than 0.1 Hz or negative.");
        publishFrameRate_ = cameraGrabFrameRate_;
    }
    RCLCPP_INFO_STREAM(get_logger(), " * [DYN] Publish framerate [Hz]:  " << publishFrameRate_);
}

void ZedCameraComponent::callback_updateDiagnostic(diagnostic_updater::DiagnosticStatusWrapper& diagnosticStatus)
{

    RCLCPP_DEBUG(get_logger(),"*** Update Diagnostic ***");

    if (connectionStatus_ != sl::ERROR_CODE::SUCCESS) {
        diagnosticStatus.summary(diagnostic_msgs::msg::DiagnosticStatus::ERROR, sl::toString(connectionStatus_).c_str());
        return;
    }

    if (grabStatus_ == sl::ERROR_CODE::SUCCESS) {
        double freq = 1. / grabPeriodMean_sec_->getAvg();
        double freq_perc = 100. * freq / publishFrameRate_;
        diagnosticStatus.addf("Capture", "Mean Frequency: %.1f Hz (%.1f%%)", freq, freq_perc);

        double frame_proc_sec = elapsedPeriodMean_sec_->getAvg();
        //double frame_grab_period = 1. / mCamGrabFrameRate;
        double frame_grab_period = 1. / publishFrameRate_;
        diagnosticStatus.addf(
            "Capture", "Tot. Processing Time: %.6f sec (Max. %.3f sec)", frame_proc_sec,
            frame_grab_period);


        if (frame_proc_sec > frame_grab_period) {
            systemOverloadCount_++;
        }

        if (systemOverloadCount_ >= 10) {
            diagnosticStatus.summary(
                diagnostic_msgs::msg::DiagnosticStatus::WARN,
                "System overloaded. Consider reducing 'general.pub_frame_rate' or 'general.grab_resolution'");
        } else {
            systemOverloadCount_ = 0;
            diagnosticStatus.summary(diagnostic_msgs::msg::DiagnosticStatus::OK, "Camera grabbing");
        }

        diagnosticStatus.add("Input mode", "Live Camera");

    } else {
        diagnosticStatus.summaryf(
            diagnostic_msgs::msg::DiagnosticStatus::ERROR, "Camera error: %s",
            sl::toString(grabStatus_).c_str());
    }

    if (cameraModel_ == sl::MODEL::ZED2 || cameraModel_ == sl::MODEL::ZED2i) {
        diagnosticStatus.addf("Left CMOS Temp.", "%.1f °C", temperatureLeft_);
        diagnosticStatus.addf("Right CMOS Temp.", "%.1f °C", temperatureRight_);

        if (temperatureLeft_ > 70.f || temperatureRight_ > 70.f) {
            diagnosticStatus.summary(diagnostic_msgs::msg::DiagnosticStatus::WARN, "High Camera temperature");
        }
    } else {
        diagnosticStatus.add("Left CMOS Temp.", "N/A");
        diagnosticStatus.add("Right CMOS Temp.", "N/A");
    }
}


//void ZedCameraComponent::getAdvancedParams()
//{
//    rclcpp::Parameter paramVal;
//    std::string paramName;
//
//    rcl_interfaces::msg::ParameterDescriptor read_only_descriptor;
//    read_only_descriptor.read_only = true;
//
//    RCLCPP_INFO(get_logger(), "*** Advanced parameters ***");
//
//    getParam(
//        "advanced.thread_sched_policy", threadSchedulePolicy_, threadSchedulePolicy_,
//        " * Thread sched. policy: ");
//
//    if (threadSchedulePolicy_ == "SCHED_FIFO" || threadSchedulePolicy_ == "SCHED_RR") {
//        if (!sl_tools::checkRoot()) {
//            RCLCPP_WARN_STREAM(
//                get_logger(),
//                "'sudo' permissions required to set " << mThreadSchedPolicy <<
//                    " thread scheduling policy. Using Linux default [SCHED_OTHER]");
//            mThreadSchedPolicy = "SCHED_OTHER";
//        } else {
//            getParam(
//                "advanced.thread_grab_priority", mThreadPrioGrab, mThreadPrioGrab,
//                " * Grab thread priority: ");
//            getParam(
//                "advanced.thread_sensor_priority", mThreadPrioSens, mThreadPrioSens,
//                " * Sensors thread priority: ");
//            getParam(
//                "advanced.thread_pointcloud_priority", mThreadPrioPointCloud, mThreadPrioPointCloud,
//                " * Point Cloud thread priority: ");
//        }
//    }
//}

void ZedCameraComponent::Run() {}

} // namespace perception
} // namespace nav

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(nav::perception::ZedCameraComponent);

// #include "class_loader/register_macro.hpp"
//
// CLASS_LOADER_REGISTER_CLASS(MyCustomNodeFactory, rclcpp_components::NodeFactory)
