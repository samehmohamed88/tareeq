#pragma once

namespace platform::sensors::imu {

// IMU data fields like acceleration, gyroscope, magnetometer, etc.
// {"temp":34.1484375,"roll":0.769414961,"pitch":2.975819826,"yaw":42.60821533,"acce_X":-886,"acce_Y":480,"acce_Z":16406,
//"gyro_X":-10,"gyro_Y":-102,"gyro_Z":-43,"magn_X":-79,"magn_Y":105,"magn_Z":-528}




struct IMUData
{
    const double temp;
    const double roll;
    const double pitch;
    const double yaw;
    const double acceleration_X;
    const double acceleration_Y;
    const double acceleration_Z;
    const double gyro_X;
    const double gyro_Y;
    const double gyro_Z;
    const double magnetometer_X;
    const double magnetometer_Y;
    const double magnetometer_Z;

    IMUData(double temp,
            double roll,
            double pitch,
            double yaw,
            double acceleration_X,
            double acceleration_Y,
            double acceleration_Z,
            double gyro_X,
            double gyro_Y,
            double gyro_Z,
            double magnetometer_X,
            double magnetometer_Y,
            double magnetometer_Z)
        : temp(temp)
        , roll(roll)
        , pitch(pitch)
        , yaw(getAdjustedYaw(yaw))
        , acceleration_X(acceleration_X)
        , acceleration_Y(acceleration_Y)
        , acceleration_Z(acceleration_Z)
        , gyro_X(gyro_X)
        , gyro_Y(gyro_Y)
        , gyro_Z(gyro_Z)
        , magnetometer_X(magnetometer_X)
        , magnetometer_Y(magnetometer_Y)
        , magnetometer_Z(magnetometer_Z){};

private:
    // Function to get the adjusted yaw from the IMU
    double getAdjustedYaw(double imu_yaw) {
        // Set the initial orientation of the robot
        double initial_orientation = 1.5708;  // in radians
        // Adjust the IMU's yaw reading with the initial orientation
        double adjusted_yaw = imu_yaw + initial_orientation;

        // Normalize the yaw angle to be within the range of -π to π
        adjusted_yaw = normalizeAngle(adjusted_yaw);

        return adjusted_yaw;
    }

    // Function to normalize an angle to -π to π
    double normalizeAngle(double angle) {
        while (angle > M_PI) angle -= 2 * M_PI;
        while (angle < -M_PI) angle += 2 * M_PI;
        return angle;
    }
};

struct IMUDataWithQuaternion : IMUData
{
    struct Quaternion
    {
        double x;
        double y;
        double z;
        double w;

        Quaternion(double ox, double oy, double oz, double ow)
            : x{ox}
            , y{oy}
            , z{oz}
            , w{ow} {};
    };
    IMUDataWithQuaternion(double acceleration_X,
                          double acceleration_Y,
                          double acceleration_Z,
                          double gyro_X,
                          double gyro_Y,
                          double gyro_Z,
                          double quaternion_x,
                          double quaternion_y,
                          double quaternion_z,
                          double quaternion_w)
        : IMUData(0.0,
                  0.0,
                  0.0,
                  0.0,
                  acceleration_X,
                  acceleration_Y,
                  acceleration_Z,
                  gyro_X,
                  gyro_Y,
                  gyro_Z,
                  0,
                  0,
                  0)
        , quaternion(quaternion_x, quaternion_y, quaternion_z, quaternion_w){

          };
    Quaternion quaternion;
};

} // namespace platform::sensors::imu
