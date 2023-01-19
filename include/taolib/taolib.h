#include "drivetrain.h"
#include "threading.h"
#include "vector2.h"
#include "env.h"
#include "math.h"
#include "pid.h"

// TODO:
// - Finalize API design
// - Documentation (and tutorials/examples?)
// - Web interface for graphical auton routine planning
// - Pivot turning (turning with only one side of the drivetrain.)
// - 3 Encoder odometry
// - PROS Support

namespace tao {}