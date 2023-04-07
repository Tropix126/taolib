#pragma once

#include <cmath>

#include "v5_cpp.h"

#pragma once

namespace tao {

class TrackingWheel {
public:
    enum class DeviceType { Motor, MotorGroup, Encoder, Rotation, Triport };

    TrackingWheel(vex::motor& motor, double wheel_diameter, double gearing = 1.0);
    TrackingWheel(vex::motor_group& group, double wheel_diameter, double gearing = 1.0);
    TrackingWheel(vex::rotation& rotation, double wheel_diameter, double gearing = 1.0);
    TrackingWheel(vex::encoder& encoder, double wheel_diameter, double gearing = 1.0);
    TrackingWheel(vex::triport::port& triport, double wheel_diameter, double gearing = 1.0);
	
	DeviceType get_device_type() const;

    double get_rotation() const;

	double get_travel() const;

    double get_wheel_diameter() const;

    double get_gearing() const;

    void reset();

private:
    DeviceType type_;

    void* device_ = nullptr;

    double wheel_diameter_;
    double gearing_;
};

}