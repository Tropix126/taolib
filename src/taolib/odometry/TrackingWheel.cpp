#include <cmath>
#include <stdexcept>

#include "v5_cpp.h"

#include "taolib/odometry/TrackingWheel.h"
#include "taolib/utility/math.h"

namespace tao {

TrackingWheel::TrackingWheel(vex::motor& motor, double wheel_diameter, double gearing) :
    type_(DeviceType::Motor), device_(&motor), wheel_diameter_(wheel_diameter), gearing_(gearing) {
    if (gearing == 0) {
		throw std::invalid_argument("TrackingWheel: gearing cannot be 0.");
	}
}

TrackingWheel::TrackingWheel(vex::motor_group& group, double wheel_diameter, double gearing) :
    type_(DeviceType::MotorGroup), device_(&group), wheel_diameter_(wheel_diameter), gearing_(gearing) {
    if (gearing == 0) {
		throw std::invalid_argument("TrackingWheel: gearing cannot be 0.");
	}
}

TrackingWheel::TrackingWheel(vex::encoder& encoder, double wheel_diameter, double gearing) :
    type_(DeviceType::Encoder), device_(&encoder), wheel_diameter_(wheel_diameter) {
    if (gearing == 0) {
		throw std::invalid_argument("TrackingWheel: gearing cannot be 0.");
	}
}

TrackingWheel::TrackingWheel(vex::rotation& rotation, double wheel_diameter, double gearing) :
    type_(DeviceType::Rotation), device_(&rotation), wheel_diameter_(wheel_diameter) {
    if (gearing == 0) {
		throw std::invalid_argument("TrackingWheel: gearing cannot be 0.");
	}
}

TrackingWheel::TrackingWheel(vex::triport::port& triport, double wheel_diameter, double gearing) :
    type_(DeviceType::Rotation), device_(&triport), wheel_diameter_(wheel_diameter) {
    if (gearing == 0) {
		throw std::invalid_argument("TrackingWheel: gearing cannot be 0.");
	}
}

TrackingWheel::DeviceType TrackingWheel::get_device_type() const { return type_; }

double TrackingWheel::get_wheel_diameter() const { return wheel_diameter_; }

double TrackingWheel::get_gearing() const { return gearing_; }

double TrackingWheel::get_rotation() const {
    switch (type_) {
        case DeviceType::Motor: return static_cast<vex::motor*>(device_)->position(vex::degrees);
        case DeviceType::MotorGroup: return static_cast<vex::motor_group*>(device_)->position(vex::degrees);
        case DeviceType::Encoder: return static_cast<vex::encoder*>(device_)->position(vex::degrees);
        case DeviceType::Rotation: return static_cast<vex::rotation*>(device_)->position(vex::degrees);
        case DeviceType::Triport: return static_cast<vex::triport::port*>(device_)->rotation(vex::degrees);
        default: return 0.0;
    }
}

double TrackingWheel::get_travel() const {
    double wheel_circumference = wheel_diameter_ * math::PI;

    return (get_rotation() / 360) * gearing_ * wheel_circumference;
}

void TrackingWheel::reset() {
    switch (type_) {
        case DeviceType::Motor: static_cast<vex::motor*>(device_)->resetPosition(); break;
        case DeviceType::MotorGroup: static_cast<vex::motor_group*>(device_)->resetPosition(); break;
        case DeviceType::Encoder: static_cast<vex::encoder*>(device_)->resetRotation(); break;
        case DeviceType::Rotation: static_cast<vex::rotation*>(device_)->resetPosition(); break;
        case DeviceType::Triport: static_cast<vex::triport::port*>(device_)->resetRotation(); break;
        default: break;
    }
}

}