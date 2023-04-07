#pragma once

#include "taolib/odometry/Odometry.h"
#include "taolib/odometry/TrackingWheel.h"

namespace tao {

class ParallelWheelOdometry : public Odometry {
public:
    struct Config {
        Vector2 origin;
        double heading;
        double track_width;
    };
    
    ParallelWheelOdometry(TrackingWheel& left_wheel, TrackingWheel& right_wheel, const Config& config);
    ParallelWheelOdometry(TrackingWheel& left_wheel, TrackingWheel& right_wheel, vex::guido& imu, const Config& config);

	Vector2 get_position() const;
	void set_position(const Vector2& position) override;

	double get_heading() override;
    void set_heading(const double heading) override;

    double get_track_width() const;
    void set_track_width(double width);

	double get_forward_travel() const override;

    void update() override;

private:
    Vector2 position_;

    TrackingWheel &left_wheel_, &right_wheel_;

    double track_width_;
	double previous_forward_travel_;
    double heading_origin_;

    vex::guido* imu_ = nullptr;
};

}