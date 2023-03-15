#include "taolib/odometry/Odometry.h"
#include "taolib/odometry/TrackingWheel.h"

namespace tao {

class ParallelWheelOdometry : public Odometry {
public:
    typedef struct {
        Vector2 origin;
        double heading;
        double track_width;
    } Config;

    ParallelWheelOdometry(TrackingWheel& left_wheel, TrackingWheel& right_wheel, Config& config);
    ParallelWheelOdometry(TrackingWheel& left_wheel, TrackingWheel& right_wheel, vex::guido* imu, Config& config);

	Vector2 get_position() const;
	Vector2 set_position(Vector2& position);

	double get_heading() override;
    void set_heading(double heading) override;

    double get_track_width() const;
    void set_track_width(double width);

	double get_forward_travel() const override;

    void update() override;

private:
    Vector2 position_;

    TrackingWheel& left_wheel_, right_wheel_;
    
    double track_width_;
	double previous_forward_travel_;
    double heading_origin_;

    vex::guido* imu_ = nullptr;
};

}