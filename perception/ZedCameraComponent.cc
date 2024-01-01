#include "perception/ZedCameraComponent.h"

#include "component/Component.h"
#include "rclcpp/node_options.hpp"

#include <sl/Camera.hpp>

namespace nav {
namespace perception {

ZedCameraComponent::ZedCameraComponent(const rclcpp::NodeOptions& options)
    : component::Component("", options)
    , diagnosticUpdater_(this)
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
