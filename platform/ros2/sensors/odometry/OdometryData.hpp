#pragma once

#include <array>

namespace platform::sensors::odometry {

struct OdometryData
{
private:
    struct Position
    {
        friend class OdometryData;

    private:
        double x_;
        double y_;
        double z_;

        Position(double x, double y, double z)
            : x_{x}
            , y_{y}
            , z_{z}
        {}
    };

    struct Orientation
    {
        friend class OdometryData;

    private:
        double x_;
        double y_;
        double z_;
        double w_;

        Orientation(double x, double y, double z, double w)
            : x_{x}
            , y_{y}
            , z_{z}
            , w_{w}
        {}
    };

public:
    OdometryData(double x_Position,
                 double y_Position,
                 double z_Position,
                 double x_Orientation,
                 double y_Orientation,
                 double z_Orientation,
                 double w_Orientation)
        : position_{x_Position, y_Position, z_Position}
        , orientation_{x_Orientation, y_Orientation, z_Orientation, w_Orientation}
    {}

    std::array<double, 3> getPosition() const { return {position_.x_, position_.y_, position_.z_}; }

    std::array<double, 4> getOrientation() const
    {
        return {orientation_.x_, orientation_.y_, orientation_.z_, orientation_.w_};
    }

private:
    Position position_;
    Orientation orientation_;
};

} // namespace platform::sensors::odometry
