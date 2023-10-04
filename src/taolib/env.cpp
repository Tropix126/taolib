/**
 * @file src/taolib/env.cpp
 * @author Tropical
 *
 * Defines platform-speccific methods and classes for
 * simpler cross-compatiblity between VEXcode and PROS.
 */
#include "taolib/env.h"
#include <cstdint>

namespace tao {
namespace env {

#ifdef TAO_ENV_VEXCODE
void motor_group_set_voltage(MotorGroup& group, double voltage) {
	return group.spin(vex::forward, voltage, vex::percent);
}
#elif defined(TAO_ENV_PROS)
void motor_group_set_voltage(MotorGroup& group, double voltage) {
	return group.move((voltage / 12.0) * 127.0);
}
#endif

}
}