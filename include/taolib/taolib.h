#include "drivetrain.h"
#include "utility.h"
#include "threading.h"
#include "pid.h"
#include "vector2.h"

// TODO:
// - Finalize API design
//   - Non-blocking pure pursusit (+performing actions once the drivetrain has passed a waypoint.)
//   - Moving to a point backwards always?
// - Documentation (and tutorials/examples)
// - Web interface for graphical auton routine planning
// - Pivot turning (turning with only one side of the drivetrain.)
// - 3 Encoder odometry (left/right/center)
//   - Useful for peeps without traction wheels or locked omnis who can easily slide sideways
// - PROS Support
//   - I strongly dislike parts of the PROS API, which is why I ditched this previously after
//	   it was mostly finished. Not sure if this should be a separate project or not, or how
//     it'll affect documentation.
// - Slew rate limiting
//   - Last time I messed with voltage control, I burnt out 2 motors in the span of a second.
//     I'm 90% sure that non-rev10 motors can handle basic PID control, but I want to be safe.
namespace tao {}