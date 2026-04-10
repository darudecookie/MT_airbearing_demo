#pragma once

#include <imumaths.h>

namespace gnc_math
{
    namespace conn
    {
        class BDotController
        {
            const double _negative_k;

        public:
            BDotController(const double k) : _negative_k(-k) {}

            imu::Vector<3> compute_torque(const imu::Vector<3> &B_dot) const;
        };
    } // namespace conn
} // namespace gnc_math
