#pragma once

namespace tao {
    class MotionController {
    public:
        virtual double update(double error, double delta_time) = 0;
    };
}