#include "perception/ZedCameraComponent.h"

#include "component/Component.h"
#include "rclcpp/node_options.hpp"

#include <sl/Camera.hpp>

namespace nav {
namespace perception {

ZedCameraComponent::ZedCameraComponent(const rclcpp::NodeOptions& options)
    : component::Component("", options)
{}

bool ZedCameraComponent::Init()
{
    // Parameters initialization
    initParameters();

    // ----> Diagnostic initialization
    diagnosticUpdater_.add("ZED Diagnostic", this, &ZedCameraComponent::callback_updateDiagnostic);
    std::string hw_id = std::string("Stereolabs camera: ") + cameraName_;
    diagnosticUpdater_.setHardwareID(hw_id);
    // <---- Diagnostic initialization

    // Services initialization
    initServices();

    // ----> Start camera
//    if (!startCamera()) {
//        exit(EXIT_FAILURE);
//    }
    // <---- Start camera

    // Dynamic parameters callback
//    mParamChangeCallbackHandle =
//        add_on_set_parameters_callback(std::bind(&ZedCameraComponent::callback_paramChange, this, _1));
    return true;
}

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

void ZedCamera::callback_updateDiagnostic(diagnostic_updater::DiagnosticStatusWrapper & stat)
{

    DEBUG_COMM("*** Update Diagnostic ***");

    if (mConnStatus != sl::ERROR_CODE::SUCCESS) {
        stat.summary(diagnostic_msgs::msg::DiagnosticStatus::ERROR, sl::toString(mConnStatus).c_str());
        return;
    }

    if (mGrabStatus == sl::ERROR_CODE::SUCCESS) {
        double freq = 1. / mGrabPeriodMean_sec->getAvg();
        double freq_perc = 100. * freq / mPubFrameRate;
        stat.addf("Capture", "Mean Frequency: %.1f Hz (%.1f%%)", freq, freq_perc);

        double frame_proc_sec = mElabPeriodMean_sec->getAvg();
        //double frame_grab_period = 1. / mCamGrabFrameRate;
        double frame_grab_period = 1. / mPubFrameRate;
        stat.addf(
            "Capture", "Tot. Processing Time: %.6f sec (Max. %.3f sec)", frame_proc_sec,
            frame_grab_period);


        if (frame_proc_sec > frame_grab_period) {
            mSysOverloadCount++;
        }

        if (mSysOverloadCount >= 10) {
            stat.summary(
                diagnostic_msgs::msg::DiagnosticStatus::WARN,
                "System overloaded. Consider reducing 'general.pub_frame_rate' or 'general.grab_resolution'");
        } else {
            mSysOverloadCount = 0;
            stat.summary(diagnostic_msgs::msg::DiagnosticStatus::OK, "Camera grabbing");
        }

        if (mSimMode) {
            stat.add("Input mode", "SIMULATION");
        } else if (mSvoMode) {
            stat.add("Input mode", "SVO");
        } else {
            stat.add("Input mode", "Live Camera");
        }

        if (mVdPublishing) {
            freq = 1. / mVideoDepthPeriodMean_sec->getAvg();
            freq_perc = 100. * freq / mPubFrameRate;
            double frame_grab_period = 1. / mPubFrameRate;
            stat.addf("Video/Depth", "Mean Frequency: %.1f Hz (%.1f%%)", freq, freq_perc);
            stat.addf(
                "Video/Depth", "Processing Time: %.6f sec (Max. %.3f sec)",
                mVideoDepthElabMean_sec->getAvg(), frame_grab_period);
        } else {
            stat.add("Video/Depth", "Topics not subscribed");
        }

        if (mSvoMode) {
            int frame = mZed.getSVOPosition();
            int totFrames = mZed.getSVONumberOfFrames();
            double svo_perc = 100. * (static_cast<double>(frame) / totFrames);

            stat.addf("Playing SVO", "Frame: %d/%d (%.1f%%)", frame, totFrames, svo_perc);
        }

        if (isDepthRequired()) {
            stat.add("Depth status", "ACTIVE");
            stat.add("Depth mode", sl::toString(mDepthMode).c_str());

            if (mPcPublishing) {
                double freq = 1. / mPcPeriodMean_sec->getAvg();
                double freq_perc = 100. * freq / mPcPubRate;
                stat.addf("Point Cloud", "Mean Frequency: %.1f Hz (%.1f%%)", freq, freq_perc);
                stat.addf(
                    "Point Cloud", "Processing Time: %.3f sec (Max. %.3f sec)", mPcProcMean_sec->getAvg(),
                    1. / mPcPubRate);
            } else {
                stat.add("Point Cloud", "Topic not subscribed");
            }

            if (mFloorAlignment) {
                if (mInitOdomWithPose) {
                    stat.add("Floor Detection", "NOT INITIALIZED");
                } else {
                    stat.add("Floor Detection", "INITIALIZED");
                }
            }

            if (mGnssFusionEnabled) {
                stat.addf("Fusion status", sl::toString(mFusionStatus).c_str());
                if (mPosTrackingStarted) {
                    stat.addf("GNSS Tracking status", "%s", sl::toString(mGeoPoseStatus).c_str());
                }
                if (mGnssMsgReceived) {
                    double freq = 1. / mGnssFix_sec->getAvg();
                    stat.addf("GNSS input", "Mean Frequency: %.1f Hz", freq);
                    stat.addf("GNSS Service", "%s", mGnssService.c_str());
                    if (mGnssFixValid) {
                        stat.add("GNSS Status", "FIX OK");
                    } else {
                        stat.add("GNSS Status", "NO FIX");
                    }
                } else {
                    stat.add("GNSS Fusion", "GNSS Data not available");
                }
            } else {
                stat.add("GNSS Fusion", "DISABLED");
            }

            if (mPosTrackingStarted) {
                stat.addf(
                    "Pos. Tracking status [Pose]", "%s",
                    sl::toString(mPosTrackingStatusWorld).c_str());
                stat.addf(
                    "Pos. Tracking status [Odometry]", "%s", sl::toString(
                                                                 mPosTrackingStatusCamera).c_str());

                if (mPublishTF) {
                    double freq = 1. / mPubOdomTF_sec->getAvg();
                    stat.addf("TF Odometry", "Mean Frequency: %.1f Hz", freq);

                    if (mPublishMapTF) {
                        double freq = 1. / mPubPoseTF_sec->getAvg();
                        stat.addf("TF Pose", "Mean Frequency: %.1f Hz", freq);
                    } else {
                        stat.add("TF Pose", "DISABLED");
                    }
                } else {
                    stat.add("TF Odometry", "DISABLED");
                    stat.add("TF Pose", "DISABLED");
                }
            } else {
                stat.add("Pos. Tracking status", "INACTIVE");
            }

            if (mObjDetRunning) {
                if (mObjDetSubscribed) {
                    double freq = 1. / mObjDetPeriodMean_sec->getAvg();
                    double freq_perc = 100. * freq / mPubFrameRate;
                    double frame_grab_period = 1. / mPubFrameRate;
                    stat.addf("Object detection", "Mean Frequency: %.3f Hz  (%.1f%%)", freq, freq_perc);
                    stat.addf(
                        "Object detection", "Processing Time: %.3f sec (Max. %.3f sec)",
                        mObjDetElabMean_sec->getAvg(), frame_grab_period);
                } else {
                    stat.add("Object Detection", "Active, topic not subscribed");
                }
            } else {
                stat.add("Object Detection", "INACTIVE");
            }

            if (mBodyTrkRunning) {
                if (mBodyTrkSubscribed) {
                    double freq = 1. / mBodyTrkPeriodMean_sec->getAvg();
                    double freq_perc = 100. * freq / mPubFrameRate;
                    double frame_grab_period = 1. / mPubFrameRate;
                    stat.addf("Body Tracking", "Mean Frequency: %.3f Hz  (%.1f%%)", freq, freq_perc);
                    stat.addf(
                        "Body Tracking", "Processing Time: %.3f sec (Max. %.3f sec)",
                        mBodyTrkElabMean_sec->getAvg(), frame_grab_period);
                } else {
                    stat.add("Body Tracking", "Active, topic not subscribed");
                }
            } else {
                stat.add("Body Tracking", "INACTIVE");
            }
        } else {
            stat.add("Depth status", "INACTIVE");
        }

        if (mPublishImuTF) {
            double freq = 1. / mPubImuTF_sec->getAvg();
            stat.addf("TF IMU", "Mean Frequency: %.1f Hz", freq);
        } else {
            stat.add("TF IMU", "DISABLED");
        }
    } else {
        stat.summaryf(
            diagnostic_msgs::msg::DiagnosticStatus::ERROR, "Camera error: %s",
            sl::toString(mGrabStatus).c_str());
    }

    if (mImuPublishing) {
        double freq = 1. / mImuPeriodMean_sec->getAvg();
        stat.addf("IMU", "Mean Frequency: %.1f Hz", freq);
    } else {
        stat.add("IMU Sensor", "Topics not subscribed");
    }

    if (!mSvoMode && !mSimMode && sl_tools::isZED2OrZED2i(mCamRealModel)) {
        if (mMagPublishing) {
            double freq = 1. / mMagPeriodMean_sec->getAvg();
            stat.addf("Magnetometer", "Mean Frequency: %.1f Hz", freq);
        } else {
            stat.add("Magnetometer Sensor", "Topics not subscribed");
        }
    } else {
        stat.add("Magnetometer Sensor", "N/A");
    }

    if (!mSvoMode && !mSimMode && sl_tools::isZED2OrZED2i(mCamRealModel)) {
        if (mBaroPublishing) {
            double freq = 1. / mBaroPeriodMean_sec->getAvg();
            stat.addf("Barometer", "Mean Frequency: %.1f Hz", freq);
        } else {
            stat.add("Barometer Sensor", "Topics not subscribed");
        }
    } else {
        stat.add("Barometer Sensor", "N/A");
    }

    if (!mSvoMode && !mSimMode && sl_tools::isZED2OrZED2i(mCamRealModel)) {
        stat.addf("Left CMOS Temp.", "%.1f °C", mTempLeft);
        stat.addf("Right CMOS Temp.", "%.1f °C", mTempRight);

        if (mTempLeft > 70.f || mTempRight > 70.f) {
            stat.summary(diagnostic_msgs::msg::DiagnosticStatus::WARN, "High Camera temperature");
        }
    } else {
        stat.add("Left CMOS Temp.", "N/A");
        stat.add("Right CMOS Temp.", "N/A");
    }

    if (!mSvoMode && !mSimMode && sl_tools::isZEDX(mCamRealModel)) {
        stat.addf("Camera Temp.", "%.1f °C", mTempImu);

        if (mTempImu > 70.f) {
            stat.summary(diagnostic_msgs::msg::DiagnosticStatus::WARN, "High Camera temperature");
        }
    }

    if (mRecording) {
        if (!mRecStatus.status) {
            // if (mGrabActive)
            {
                stat.add("SVO Recording", "ERROR");
                stat.summary(
                    diagnostic_msgs::msg::DiagnosticStatus::WARN,
                    "Error adding frames to SVO file while recording. "
                    "Check "
                    "free disk space");
            }
        } else {
            stat.add("SVO Recording", "ACTIVE");
            stat.addf("SVO compression time", "%g msec", mRecStatus.average_compression_time);
            stat.addf("SVO compression ratio", "%.1f%%", mRecStatus.average_compression_ratio);
        }
    } else {
        stat.add("SVO Recording", "NOT ACTIVE");
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
