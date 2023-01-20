/**
 * @file src/taolib/env.h
 * @author Tropical
 *
 * Defines platform-speccific methods and classes for
 * simpler cross-compatiblity between VEXcode and PROS.
 */

#include "taolib/env.h"

namespace tao {
namespace env {

void sleep_for(uint32_t time) {
	#ifdef TAO_ENV_VEXCODE
		vex::this_thread::sleep_for(time);
	#elif defined(TAO_ENV_PROS)
		pros::delay(time);
	#endif
}

uint64_t system_time_high_resolution() {
	#ifdef TAO_ENV_VEXCODE
		return vex::timer::systemHighResolution();
	#elif defined(TAO_ENV_PROS)
		return pros::micros();
	#endif
}

}
}