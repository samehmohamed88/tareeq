//#pragma once
//
//#include "platform/sensors/ImuData.hpp"
//
//#include <Eigen/Dense>
//#include <Eigen/Geometry>
//#include <nlohmann/json.hpp>
//
//#include <cmath>
//#include <string>
//
//namespace platform::sensors {
//
//class SensorDataFactories
//{
//public:
//    /// @brief Converts Euler angles (roll, pitch, yaw) to a quaternion.
//    /// @param roll The roll angle in radians.
//    /// @param pitch The pitch angle in radians.
//    /// @param yaw The yaw angle in radians.
//    /// @return A quaternion representing the rotation.
//    static Eigen::Quaterniond eulerToQuaternion(double roll, double pitch, double yaw)
//    {
//        Eigen::AngleAxisd rollAngle(roll, Eigen::Vector3d::UnitX());
//        Eigen::AngleAxisd pitchAngle(pitch, Eigen::Vector3d::UnitY());
//        Eigen::AngleAxisd yawAngle(yaw, Eigen::Vector3d::UnitZ());
//
//        Eigen::Quaterniond q = yawAngle * pitchAngle * rollAngle;
//        return q;
//    }
//
//    /// @brief Creates an IMU data object from a JSON string.
//    /// @param jsonString The JSON string representing the IMU data.
//    /// @return A populated ImuData object.
//    static ImuData createImuDataFromJsonString(const std::string& jsonString)
//    {
//
//        // Parse the JSON string
//        auto json = nlohmann::json::parse(jsonString);
//        auto quaternion = eulerToQuaternion(json["r"].get<double>() * (M_PI / 180.0),
//                                            json["p"].get<double>() * (M_PI / 180.0),
//                                            json["w"].get<double>() * (M_PI / 180.0));
//
//        return
//        {
//            quaternion,
//                Eigen::Vector3d{
//                    (json["ax"].get<double>() / accelerometerScaleFactor_) * g_, // gets acceleration in m/s^2
//                    (json["ay"].get<double>() / accelerometerScaleFactor_) * g_, // gets acceleration in m/s^2
//                    (json["az"].get<double>() / accelerometerScaleFactor_) * g_, // gets acceleration in m/s^2}
//                },
//                Eigen::Vector3d
//            {
//                json["gx"].get<double>() * (M_PI / 180.0), json["gy"].get<double>() * (M_PI / 180.0),
//                    json["gz"].get<double>() * (M_PI / 180.0)
//            }
//        }
//    }
//
//private:
//    /// this is empirically found by taking 20 measurements
//    /// of the IMU stationary on a flat even table
//    static constexpr double accelerometerScaleFactor_ = 1531.80;
//    static constexpr double g_ = 9.81; ///< Acceleration due to gravity in m/s^2
//};
//
//} // namespace platform::sensors
