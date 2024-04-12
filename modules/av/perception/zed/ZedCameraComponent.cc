#include "perception/zed/ZedCameraComponent.h"
#include "component/StopWatch.h"
#include "component/Component.h"

#include <rclcpp/node_options.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/distortion_models.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <image_transport/camera_publisher.hpp>
#include <image_transport/image_transport.hpp>
#include <diagnostic_updater/diagnostic_updater.hpp>
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
    : rclcpp::Node("ZedCameraComponent", options)
    , diagnosticUpdater_{this}
    , videoQos_{1}
    , grabFreqTimer_{get_clock()}
    , imagePublisherFrequencyTimer_{get_clock()}
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

        RCLCPP_WARN(get_logger(), "Errors opening camera: %s", sl::toString(connectionStatus_).c_str());

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
    leftCameraInfoMessage_ = std::make_shared<sensor_msgs::msg::CameraInfo>();
    rightCameraInfoMessage_ = std::make_shared<sensor_msgs::msg::CameraInfo>();
    leftCameraInfoRawMessage_ = std::make_shared<sensor_msgs::msg::CameraInfo>();
    rightCameraInfoRawMessage_ = std::make_shared<sensor_msgs::msg::CameraInfo>();

//    setTFCoordFrameNames();  // Requires mZedRealCamModel available only after camera opening

    leftCameraFrameId_ = cameraName_ + "_left_camera_optical_frame";
    rightCameraFrameId_ = cameraName_ + "_right_camera_optical_frame";

    setupCameraInfoMessages(zed_, leftCameraInfoMessage_, rightCameraInfoMessage_, leftCameraFrameId_, rightCameraFrameId_);
    setupCameraInfoMessages(
        zed_, leftCameraInfoRawMessage_, rightCameraInfoRawMessage_, leftCameraFrameId_, rightCameraFrameId_, true);
    // <---- Camera Info messages

    // Requires mZedRealCamModel available only after camera opening
    initPublishers();

    // Disable AEC_AGC and Auto Whitebalance to trigger it if user set it to
    // automatic
    zed_.setCameraSettings(sl::VIDEO_SETTINGS::AEC_AGC, 0);
    zed_.setCameraSettings(sl::VIDEO_SETTINGS::WHITEBALANCE_AUTO, 0);
    // Force parameters with a dummy grab
    zed_.grab();

    // Initialize timestamp to avoid wrong initial data
    frameTimestamp_ = slTime2Ros(zed_.getTimestamp(sl::TIME_REFERENCE::IMAGE));

    // ----> Initialize Diagnostic statistics
    elapsedPeriodMean_sec_ = std::make_unique<WindowAverage>(cameraGrabFrameRate_);
    grabPeriodMean_sec_ = std::make_unique<WindowAverage>(cameraGrabFrameRate_);
    // <---- Initialize Diagnostic statistics

    // Init and start threads
    initThreads();

    return true;
}

void ZedCameraComponent::initThreads()
{
    // Start grab thread
    grabThread_ = std::thread(&ZedCameraComponent::threadFunc_zedGrab, this);
}

void ZedCameraComponent::threadFunc_zedGrab()
{
    RCLCPP_DEBUG(get_logger(), "Grab thread started");

    frameCount_ = 0;

    // ----> Grab Runtime parameters
    runtimeParameters_.enable_depth = false;
    runtimeParameters_.measure3D_reference_frame = sl::REFERENCE_FRAME::CAMERA;
//    runtimeParameters_.remove_saturated_areas = true;
    // <---- Grab Runtime parameters

    // Infinite grab thread
    while (1) {

        component::StopWatch grabElapsedTimer(get_clock());

        // ----> Interruption check
        if (!rclcpp::ok()) {
            RCLCPP_DEBUG(get_logger(), "Ctrl+C received: stopping grab thread");
            break;
        }
        if (threadStop_) {
            RCLCPP_DEBUG(get_logger(), "Grab thread stopped");
            break;
        }
        // <---- Interruption check

        // ----> Grab freq calculation
        double elapsed_sec = grabFreqTimer_.toc();
        grabPeriodMean_sec_->addValue(elapsed_sec);
        grabFreqTimer_.tic();


        // Start processing timer for diagnostic
        grabElapsedTimer.tic();

        // ZED grab
        grabStatus_ = zed_.grab(runtimeParameters_);

        // ----> Grab errors?
        // Note: disconnection are automatically handled by the ZED SDK
        if (grabStatus_ != sl::ERROR_CODE::SUCCESS) {
            if (grabStatus_ == sl::ERROR_CODE::CAMERA_REBOOTING) {
                RCLCPP_ERROR_STREAM(
                    get_logger(),
                    "Connection issue detected: " << sl::toString(grabStatus_).c_str());
                rclcpp::sleep_for(1000ms);
                continue;
            } else {
                RCLCPP_ERROR_STREAM(
                    get_logger(), "Critical camera error: " << sl::toString(
                        grabStatus_).c_str() << ". Node stopped.");
                break;
            }
        }
        // <---- Grab errors?
        frameCount_++;

        // ----> Timestamp
        frameTimestamp_ = slTime2Ros(zed_.getTimestamp(sl::TIME_REFERENCE::IMAGE));
        // <---- Timestamp

        // ----> Retrieve Image data if someone has subscribed
        // Retrieve data if there are subscriber to topics
        if (areVideoTopicsSubscribed()) {
            RCLCPP_DEBUG_STREAM(get_logger(), "Retrieving video/depth data");
            retrieveImages();

            rclcpp::Time publishTimestamp;
            publishImages(publishTimestamp);
        }
        // <---- Retrieve Image data if someone has subscribed

        // Diagnostic statistics update
        double meanElapsedSec = elapsedPeriodMean_sec_->addValue(grabElapsedTimer.toc());
    }

    RCLCPP_DEBUG_STREAM(get_logger(), "Grab thread finished");
}

bool ZedCameraComponent::areVideoTopicsSubscribed() {
    leftNumberSubscribed_ = 0;
    leftRawNumberSubscribed_ = 0;
    rightNumberSubscribed_ = 0;
    rightRawNumberSubscribed_ = 0;

    try {
        leftNumberSubscribed_ = leftImagePublisher_.getNumSubscribers();
        leftRawNumberSubscribed_ = leftRawImagePublisher_.getNumSubscribers();
        rightNumberSubscribed_ = rightImagePublisher_.getNumSubscribers();
        rightRawNumberSubscribed_ = rightRawImagePublisher_.getNumSubscribers();

    } catch (...) {
        rcutils_reset_error();
        RCLCPP_DEBUG_STREAM(get_logger(), "publishImages: Exception while counting subscribers");
        return false;
    }

    return (leftNumberSubscribed_ +
            leftRawNumberSubscribed_ +
            rightNumberSubscribed_ +
            rightRawNumberSubscribed_) > 0;
}

void ZedCameraComponent::retrieveImages() {
    bool retrieved = false;

    // ----> Retrieve all required data
    RCLCPP_DEBUG_STREAM(get_logger(), "Retrieving Video Data");
    if (leftNumberSubscribed_ > 0) {
        retrieved |= sl::ERROR_CODE::SUCCESS ==
                     zed_.retrieveImage(matrixLeftImage_, sl::VIEW::LEFT, sl::MEM::CPU, matrixResolution_);
        mSdkGrabTS = matrixLeftImage_.timestamp;
    }
    if (leftRawNumberSubscribed_ > 0) {
        retrieved |=
                sl::ERROR_CODE::SUCCESS ==
                zed_.retrieveImage(matrixLefImageRaw_, sl::VIEW::LEFT_UNRECTIFIED, sl::MEM::CPU, matrixResolution_);
        mSdkGrabTS = matrixLefImageRaw_.timestamp;
    }
    if (rightNumberSubscribed_ > 0) {
        retrieved |= sl::ERROR_CODE::SUCCESS ==
                     zed_.retrieveImage(matrixRightImage_, sl::VIEW::RIGHT, sl::MEM::CPU, matrixResolution_);
        mSdkGrabTS = matrixRightImage_.timestamp;
    }
    if (rightRawNumberSubscribed_ > 0) {
        retrieved |= sl::ERROR_CODE::SUCCESS ==
                     zed_.retrieveImage(
                             matrixRightImageRaw_, sl::VIEW::RIGHT_UNRECTIFIED, sl::MEM::CPU, matrixResolution_);
        mSdkGrabTS = matrixRightImageRaw_.timestamp;
    }
    RCLCPP_DEBUG_STREAM(get_logger(), "Video Data retrieved");
    // <---- Retrieve all required data
}

void ZedCameraComponent::publishImages(rclcpp::Time & publishTimestamp) {
    RCLCPP_DEBUG(get_logger(), "*** Publish Image topics *** ");
    component::StopWatch elapsedTimer(get_clock());

    // Start processing timer for diagnostic
    elapsedTimer.tic();

    // ----> Check if a grab has been done before publishing the same images
    if (mSdkGrabTS.data_ns == lastGrabTimestamp_.data_ns) {
        publishTimestamp = TIMEZERO_ROS;
        // Data not updated by a grab calling in the grab thread
        RCLCPP_DEBUG_STREAM(get_logger(), "publishVideo: ignoring not update data");
        return;
    }

    if (lastGrabTimestamp_.data_ns != 0) {
        double period_sec = static_cast<double>(mSdkGrabTS.data_ns - lastGrabTimestamp_.data_ns) / 1e9;
        RCLCPP_DEBUG_STREAM(get_logger(),
                "VIDEO/DEPTH PUB LAST PERIOD: " << period_sec << " sec @" << 1. / period_sec << " Hz");

        videoPeriodMean_sec_->addValue(period_sec);
        RCLCPP_DEBUG_STREAM(get_logger(),
                "VIDEO/DEPTH PUB MEAN PERIOD: " << videoPeriodMean_sec_->getAvg() << " sec @"
                                                << 1. / videoPeriodMean_sec_->getAvg() << " Hz");
    }
    lastGrabTimestamp_ = mSdkGrabTS;
    // <---- Check if a grab has been done before publishing the same images

    publishTimestamp = slTime2Ros(mSdkGrabTS, get_clock()->get_clock_type());
    // ----> Publish the left=rgb image if someone has subscribed to
    if (leftNumberSubscribed_ > 0) {
        RCLCPP_DEBUG_STREAM(get_logger(),"leftNumberSubscribed_: " << leftNumberSubscribed_);
        publishImageWithInfo(matrixLeftImage_, leftImagePublisher_, leftCameraInfoMessage_, leftCameraFrameId_, publishTimestamp);
    }
    // <---- Publish the left=rgb image if someone has subscribed to

    // ----> Publish the left_raw=rgb_raw image if someone has subscribed to
    if (leftRawNumberSubscribed_ > 0) {
        RCLCPP_DEBUG_STREAM(get_logger(),"leftRawNumberSubscribed_: " << leftRawNumberSubscribed_);
        publishImageWithInfo(
                matrixLefImageRaw_, leftRawImagePublisher_, leftCameraInfoRawMessage_, leftCameraFrameId_, publishTimestamp);
    }
    // <---- Publish the left_raw=rgb_raw image if someone has subscribed to

    // ----> Publish the right image if someone has subscribed to
    if (rightNumberSubscribed_ > 0) {
        RCLCPP_DEBUG_STREAM(get_logger(), "mRightSubnumber: " << rightNumberSubscribed_);
        publishImageWithInfo(matrixRightImage_, rightImagePublisher_, rightCameraInfoMessage_, rightCameraFrameId_, publishTimestamp);
    }
    // <---- Publish the right image if someone has subscribed to

    // ----> Publish the right raw image if someone has subscribed to
    if (rightRawNumberSubscribed_ > 0) {
        RCLCPP_DEBUG_STREAM(get_logger(), "mRightRawSubnumber: " << rightRawNumberSubscribed_);
        publishImageWithInfo(
                matrixRightImageRaw_, rightRawImagePublisher_, rightCameraInfoRawMessage_, rightCameraFrameId_, publishTimestamp);
    }
    // <---- Publish the right raw image if someone has subscribed to

    // Diagnostic statistic
    videoElapsedMean_sec_->addValue(elapsedTimer.toc());

    // ----> Check publishing frequency
    double vd_period_usec = 1e6 / publishFrameRate_;
    double elapsed_usec = imagePublisherFrequencyTimer_.toc() * 1e6;

    if (elapsed_usec < vd_period_usec) {
        rclcpp::sleep_for(std::chrono::microseconds(static_cast<int>(vd_period_usec - elapsed_usec)));
    }
    imagePublisherFrequencyTimer_.tic();
    // <---- Check publishing frequency

    RCLCPP_DEBUG(get_logger(), "*** Image topics published *** ");
}

std::unique_ptr<sensor_msgs::msg::Image> ZedCameraComponent::imageToROSmsg(
        sl::Mat & img, std::string frameId, rclcpp::Time t)
{
    std::unique_ptr<sensor_msgs::msg::Image> imgMessage = std::make_unique<sensor_msgs::msg::Image>();

    imgMessage->header.stamp = t;
    imgMessage->header.frame_id = frameId;
    imgMessage->height = img.getHeight();
    imgMessage->width = img.getWidth();

    int num = 1;  // for endianness detection
    imgMessage->is_bigendian = !(*reinterpret_cast<char *>(&num) == 1);

    imgMessage->step = img.getStepBytes();

    size_t size = imgMessage->step * imgMessage->height;

    uint8_t * data_ptr = nullptr;

    sl::MAT_TYPE dataType = img.getDataType();

    switch (dataType) {
        case sl::MAT_TYPE::F32_C1: /**< float 1 channel.*/
            imgMessage->encoding = sensor_msgs::image_encodings::TYPE_32FC1;
            data_ptr = reinterpret_cast<uint8_t *>(img.getPtr<sl::float1>());
            imgMessage->data = std::vector<uint8_t>(data_ptr, data_ptr + size);
            break;

        case sl::MAT_TYPE::F32_C2: /**< float 2 channels.*/
            imgMessage->encoding = sensor_msgs::image_encodings::TYPE_32FC2;
            data_ptr = reinterpret_cast<uint8_t *>(img.getPtr<sl::float2>());
            imgMessage->data = std::vector<uint8_t>(data_ptr, data_ptr + size);
            break;

        case sl::MAT_TYPE::F32_C3: /**< float 3 channels.*/
            imgMessage->encoding = sensor_msgs::image_encodings::TYPE_32FC3;
            data_ptr = reinterpret_cast<uint8_t *>(img.getPtr<sl::float3>());
            imgMessage->data = std::vector<uint8_t>(data_ptr, data_ptr + size);
            break;

        case sl::MAT_TYPE::F32_C4: /**< float 4 channels.*/
            imgMessage->encoding = sensor_msgs::image_encodings::TYPE_32FC4;
            data_ptr = reinterpret_cast<uint8_t *>(img.getPtr<sl::float4>());
            imgMessage->data = std::vector<uint8_t>(data_ptr, data_ptr + size);
            break;

        case sl::MAT_TYPE::U8_C1: /**< unsigned char 1 channel.*/
            imgMessage->encoding = sensor_msgs::image_encodings::MONO8;
            data_ptr = reinterpret_cast<uint8_t *>(img.getPtr<sl::uchar1>());
            imgMessage->data = std::vector<uint8_t>(data_ptr, data_ptr + size);
            break;

        case sl::MAT_TYPE::U8_C2: /**< unsigned char 2 channels.*/
            imgMessage->encoding = sensor_msgs::image_encodings::TYPE_8UC2;
            data_ptr = reinterpret_cast<uint8_t *>(img.getPtr<sl::uchar2>());
            imgMessage->data = std::vector<uint8_t>(data_ptr, data_ptr + size);
            break;

        case sl::MAT_TYPE::U8_C3: /**< unsigned char 3 channels.*/
            imgMessage->encoding = sensor_msgs::image_encodings::BGR8;
            data_ptr = reinterpret_cast<uint8_t *>(img.getPtr<sl::uchar3>());
            imgMessage->data = std::vector<uint8_t>(data_ptr, data_ptr + size);
            break;

        case sl::MAT_TYPE::U8_C4: /**< unsigned char 4 channels.*/
            imgMessage->encoding = sensor_msgs::image_encodings::BGRA8;
            data_ptr = reinterpret_cast<uint8_t *>(img.getPtr<sl::uchar4>());
            imgMessage->data = std::vector<uint8_t>(data_ptr, data_ptr + size);
            break;
    }

    return imgMessage;
}



void ZedCameraComponent::publishImageWithInfo(
        sl::Mat & img, image_transport::CameraPublisher & imagePublisher, CameraInfoMessage & cameraInfoMessage,
        std::string imgFrameId, rclcpp::Time t)
{
    auto image = imageToROSmsg(img, imgFrameId, t);
    cameraInfoMessage->header.stamp = t;
    RCLCPP_DEBUG_STREAM(get_logger(), "Publishing IMAGE message: " << t.nanoseconds() << " nsec");
    imagePublisher.publish(std::move(image), cameraInfoMessage);
}

void ZedCameraComponent::setupCameraInfoMessages(sl::Camera& zed, std::shared_ptr<sensor_msgs::msg::CameraInfo> leftCamInfoMsg,
        std::shared_ptr<sensor_msgs::msg::CameraInfo> rightCamInfoMsg, std::string leftFrameId,
        std::string rightFrameId, bool rawParam) {

    sl::CalibrationParameters zedParam;

    if (rawParam) {
        zedParam =
            zed.getCameraInformation(matrixResolution_).camera_configuration.calibration_parameters_raw;
    } else {
        zedParam = zed.getCameraInformation(matrixResolution_).camera_configuration.calibration_parameters;
    }

    float baseline = zedParam.getCameraBaseline();

    // ROS2 order (OpenCV) -> k1,k2,p1,p2,k3,k4,k5,k6,s1,s2,s3,s4
    switch (cameraModel_) {
        case sl::MODEL::ZED: // PLUMB_BOB
            leftCamInfoMsg->distortion_model = sensor_msgs::distortion_models::PLUMB_BOB;
            rightCamInfoMsg->distortion_model = sensor_msgs::distortion_models::PLUMB_BOB;
            leftCamInfoMsg->d.resize(5);
            rightCamInfoMsg->d.resize(5);
            leftCamInfoMsg->d[0] = zedParam.left_cam.disto[0];    // k1
            leftCamInfoMsg->d[1] = zedParam.left_cam.disto[1];    // k2
            leftCamInfoMsg->d[2] = zedParam.left_cam.disto[2];    // p1
            leftCamInfoMsg->d[3] = zedParam.left_cam.disto[3];    // p2
            leftCamInfoMsg->d[4] = zedParam.left_cam.disto[4];    // k3
            rightCamInfoMsg->d[0] = zedParam.right_cam.disto[0];  // k1
            rightCamInfoMsg->d[1] = zedParam.right_cam.disto[1];  // k2
            rightCamInfoMsg->d[2] = zedParam.right_cam.disto[2];  // p1
            rightCamInfoMsg->d[3] = zedParam.right_cam.disto[3];  // p2
            rightCamInfoMsg->d[4] = zedParam.right_cam.disto[4];  // k3
            break;

        case sl::MODEL::ZED2:  // RATIONAL_POLYNOMIAL
        case sl::MODEL::ZED2i:  // RATIONAL_POLYNOMIAL
        case sl::MODEL::ZED_X:  // RATIONAL_POLYNOMIAL
        case sl::MODEL::ZED_XM:  // RATIONAL_POLYNOMIAL
            leftCamInfoMsg->distortion_model = sensor_msgs::distortion_models::RATIONAL_POLYNOMIAL;
            rightCamInfoMsg->distortion_model = sensor_msgs::distortion_models::RATIONAL_POLYNOMIAL;
            leftCamInfoMsg->d.resize(8);
            rightCamInfoMsg->d.resize(8);
            leftCamInfoMsg->d[0] = zedParam.left_cam.disto[0];    // k1
            leftCamInfoMsg->d[1] = zedParam.left_cam.disto[1];    // k2
            leftCamInfoMsg->d[2] = zedParam.left_cam.disto[2];    // p1
            leftCamInfoMsg->d[3] = zedParam.left_cam.disto[3];    // p2
            leftCamInfoMsg->d[4] = zedParam.left_cam.disto[4];    // k3
            leftCamInfoMsg->d[5] = zedParam.left_cam.disto[5];    // k4
            leftCamInfoMsg->d[6] = zedParam.left_cam.disto[6];    // k5
            leftCamInfoMsg->d[7] = zedParam.left_cam.disto[7];    // k6
            rightCamInfoMsg->d[0] = zedParam.right_cam.disto[0];  // k1
            rightCamInfoMsg->d[1] = zedParam.right_cam.disto[1];  // k2
            rightCamInfoMsg->d[2] = zedParam.right_cam.disto[2];  // p1
            rightCamInfoMsg->d[3] = zedParam.right_cam.disto[3];  // p2
            rightCamInfoMsg->d[4] = zedParam.right_cam.disto[4];  // k3
            rightCamInfoMsg->d[5] = zedParam.right_cam.disto[5];  // k4
            rightCamInfoMsg->d[6] = zedParam.right_cam.disto[6];  // k5
            rightCamInfoMsg->d[7] = zedParam.right_cam.disto[7];  // k6
            break;

        case sl::MODEL::ZED_M:
            if (zedParam.left_cam.disto[5] != 0 && // k4!=0
                zedParam.right_cam.disto[2] == 0 && // p1==0
                zedParam.right_cam.disto[3] == 0) // p2==0
            {
                leftCamInfoMsg->distortion_model = sensor_msgs::distortion_models::EQUIDISTANT;
                rightCamInfoMsg->distortion_model = sensor_msgs::distortion_models::EQUIDISTANT;

                leftCamInfoMsg->d.resize(4);
                rightCamInfoMsg->d.resize(4);
                leftCamInfoMsg->d[0] = zedParam.left_cam.disto[0];    // k1
                leftCamInfoMsg->d[1] = zedParam.left_cam.disto[1];    // k2
                leftCamInfoMsg->d[2] = zedParam.left_cam.disto[4];    // k3
                leftCamInfoMsg->d[3] = zedParam.left_cam.disto[5];    // k4
                rightCamInfoMsg->d[0] = zedParam.right_cam.disto[0];  // k1
                rightCamInfoMsg->d[1] = zedParam.right_cam.disto[1];  // k2
                rightCamInfoMsg->d[2] = zedParam.right_cam.disto[4];  // k3
                rightCamInfoMsg->d[3] = zedParam.right_cam.disto[5];  // k4
            } else {
                leftCamInfoMsg->distortion_model = sensor_msgs::distortion_models::PLUMB_BOB;
                rightCamInfoMsg->distortion_model = sensor_msgs::distortion_models::PLUMB_BOB;
                leftCamInfoMsg->d.resize(5);
                rightCamInfoMsg->d.resize(5);
                leftCamInfoMsg->d[0] = zedParam.left_cam.disto[0];  // k1
                leftCamInfoMsg->d[1] = zedParam.left_cam.disto[1];  // k2
                leftCamInfoMsg->d[2] = zedParam.left_cam.disto[2];  // p1
                leftCamInfoMsg->d[3] = zedParam.left_cam.disto[3];  // p2
                leftCamInfoMsg->d[4] = zedParam.left_cam.disto[4];  // k3
                rightCamInfoMsg->d[0] = zedParam.right_cam.disto[0]; // k1
                rightCamInfoMsg->d[1] = zedParam.right_cam.disto[1]; // k2
                rightCamInfoMsg->d[2] = zedParam.right_cam.disto[2]; // p1
                rightCamInfoMsg->d[3] = zedParam.right_cam.disto[3]; // p2
                rightCamInfoMsg->d[4] = zedParam.right_cam.disto[4]; // k3
            }
    }

    leftCamInfoMsg->k.fill(0.0);
    rightCamInfoMsg->k.fill(0.0);
    leftCamInfoMsg->k[0] = static_cast<double>(zedParam.left_cam.fx);
    leftCamInfoMsg->k[2] = static_cast<double>(zedParam.left_cam.cx);
    leftCamInfoMsg->k[4] = static_cast<double>(zedParam.left_cam.fy);
    leftCamInfoMsg->k[5] = static_cast<double>(zedParam.left_cam.cy);
    leftCamInfoMsg->k[8] = 1.0;
    rightCamInfoMsg->k[0] = static_cast<double>(zedParam.right_cam.fx);
    rightCamInfoMsg->k[2] = static_cast<double>(zedParam.right_cam.cx);
    rightCamInfoMsg->k[4] = static_cast<double>(zedParam.right_cam.fy);
    rightCamInfoMsg->k[5] = static_cast<double>(zedParam.right_cam.cy);
    rightCamInfoMsg->k[8] = 1.0;
    leftCamInfoMsg->r.fill(0.0);
    rightCamInfoMsg->r.fill(0.0);

    for (size_t i = 0; i < 3; i++) {
        // identity
        rightCamInfoMsg->r[i + i * 3] = 1;
        leftCamInfoMsg->r[i + i * 3] = 1;
    }

    if (rawParam) {
        // ROS frame (X forward, Z up, Y left)
        for (int i = 0; i < 9; i++) {
            rightCamInfoMsg->r[i] = zedParam.stereo_transform.getRotationMatrix().r[i];
        }
    }

    leftCamInfoMsg->p.fill(0.0);
    rightCamInfoMsg->p.fill(0.0);
    leftCamInfoMsg->p[0] = static_cast<double>(zedParam.left_cam.fx);
    leftCamInfoMsg->p[2] = static_cast<double>(zedParam.left_cam.cx);
    leftCamInfoMsg->p[5] = static_cast<double>(zedParam.left_cam.fy);
    leftCamInfoMsg->p[6] = static_cast<double>(zedParam.left_cam.cy);
    leftCamInfoMsg->p[10] = 1.0;
    // http://docs.ros.org/api/sensor_msgs/html/msg/CameraInfo.html
    rightCamInfoMsg->p[3] = static_cast<double>(-1 * zedParam.left_cam.fx * baseline);
    rightCamInfoMsg->p[0] = static_cast<double>(zedParam.right_cam.fx);
    rightCamInfoMsg->p[2] = static_cast<double>(zedParam.right_cam.cx);
    rightCamInfoMsg->p[5] = static_cast<double>(zedParam.right_cam.fy);
    rightCamInfoMsg->p[6] = static_cast<double>(zedParam.right_cam.cy);
    rightCamInfoMsg->p[10] = 1.0;
    leftCamInfoMsg->width = rightCamInfoMsg->width = static_cast<uint32_t>(matrixResolution_.width);
    leftCamInfoMsg->height = rightCamInfoMsg->height = static_cast<uint32_t>(matrixResolution_.height);
    leftCamInfoMsg->header.frame_id = leftFrameId;
    rightCamInfoMsg->header.frame_id = rightFrameId;
}

void ZedCameraComponent::initPublishers()
{
    RCLCPP_INFO(get_logger(), "*** PUBLISHED TOPICS ***");

    // ----> Topics names definition
    std::string rightTopicRoot = "right";
    std::string leftTopicRoot = "left";
    std::string imgTopic = "/image_rect_color";
    std::string imgRawTopic = "/image_raw_color";
    std::string rawSuffix = "_raw";
    std::string leftTopic = topicRoot_ + leftTopicRoot + imgTopic;
    std::string leftRawTopic = topicRoot_ + leftTopicRoot + rawSuffix + imgRawTopic;
    std::string rightTopic = topicRoot_ + rightTopicRoot + imgTopic;
    std::string rightRawTopic = topicRoot_ + rightTopicRoot + rawSuffix + imgRawTopic;
    // <---- Topics names definition

    // ----> Camera publishers
    leftImagePublisher_ = image_transport::create_camera_publisher(this, leftTopic, videoQos_.get_rmw_qos_profile());
    RCLCPP_INFO_STREAM(get_logger(), "Advertised on topic: " << leftImagePublisher_.getTopic());
    RCLCPP_INFO_STREAM(get_logger(), "Advertised on topic: " << leftImagePublisher_.getInfoTopic());

    leftRawImagePublisher_ = image_transport::create_camera_publisher(this, leftRawTopic, videoQos_.get_rmw_qos_profile());
    RCLCPP_INFO_STREAM(get_logger(), "Advertised on topic: " << leftRawImagePublisher_.getTopic());
    RCLCPP_INFO_STREAM(get_logger(), "Advertised on topic: " << leftRawImagePublisher_.getInfoTopic());

    rightImagePublisher_ = image_transport::create_camera_publisher(this, rightTopic, videoQos_.get_rmw_qos_profile());
    RCLCPP_INFO_STREAM(get_logger(), "Advertised on topic: " << rightImagePublisher_.getTopic());
    RCLCPP_INFO_STREAM(get_logger(), "Advertised on topic: " << rightImagePublisher_.getInfoTopic());

    rightRawImagePublisher_ = image_transport::create_camera_publisher(this, rightRawTopic, videoQos_.get_rmw_qos_profile());
    RCLCPP_INFO_STREAM(get_logger(), "Advertised on topic: " << rightRawImagePublisher_.getTopic());
    RCLCPP_INFO_STREAM(get_logger(), "Advertised on topic: " << rightRawImagePublisher_.getInfoTopic());
    // <---- Camera publishers
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
    } else if (camera_model == "zed") {
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
        //double frame_grab_period = 1. / cameraGrabFrameRate_;
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

rclcpp::Time ZedCameraComponent::slTime2Ros(sl::Timestamp t, rcl_clock_type_t clock_type)
{
    uint64_t ts_nsec = t.getNanoseconds();
    uint32_t sec = static_cast<uint32_t>(ts_nsec / 1000000000);
    uint32_t nsec = static_cast<uint32_t>(ts_nsec % 1000000000);
    return rclcpp::Time(sec, nsec, clock_type);
}

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
