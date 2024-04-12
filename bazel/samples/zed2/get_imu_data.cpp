#include <sl/Camera.hpp>

int main(int argc, char **argv) {
    // Create a ZED camera object
    sl::Camera zed;

    // Set initialization parameters
    sl::InitParameters init_params;
    init_params.camera_resolution = sl::RESOLUTION::HD720;
    init_params.camera_fps = 30;
    init_params.coordinate_units = sl::UNIT::METER;
    init_params.coordinate_system = sl::COORDINATE_SYSTEM::RIGHT_HANDED_Y_UP;

    // Open the camera
    sl::ERROR_CODE err = zed.open(init_params);
    if (err != sl::ERROR_CODE::SUCCESS) {
        std::cout << "Camera Open Error: " << sl::toString(err) << std::endl;
        zed.close();
        return 1; // Quit if an error occurred
    }

    // Check if the camera is equipped with an IMU
    if (!zed.getCameraInformation().sensors_configuration.isSensorAvailable(sl::SENSOR_TYPE::ACCELEROMETER)) {
        std::cout << "IMU not available on this camera." << std::endl;
        zed.close();
        return 1;
    }

    // Retrieve and display IMU data
    sl::SensorsData sensors_data;
    while (true) {
        if (zed.getSensorsData(sensors_data, sl::TIME_REFERENCE::CURRENT) == sl::ERROR_CODE::SUCCESS) {
            // Get the IMU data
            auto imu_data = sensors_data.imu;

            // Display the accelerometer, gyroscope, and orientation data
            std::cout << "Accelerometer: " << imu_data.linear_acceleration << std::endl;
            std::cout << "Gyroscope: " << imu_data.angular_velocity << std::endl;
            std::cout << "Orientation (Quaternion): " << imu_data.pose.getOrientation() << std::endl;
        }
    }

    // Close the camera
    zed.close();
    return 0;
}
