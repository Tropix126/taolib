#pragma once

#include "taolib/utility/Vector2.h"

namespace tao {

class Odometry {
public:
	virtual Vector2 get_position() const = 0;
    virtual void set_position(const Vector2& position) = 0;

    virtual double get_forward_travel() const = 0;

    virtual double get_heading() = 0;
    virtual void set_heading(double heading) = 0;

    virtual void update() = 0;
};

}