#pragma once

#include <imumaths.h>

namespace gnc_math
{
    class ActuatorHandler
    {
    public:
        static imu::Vector<3> possible_torque(const imu::Vector<3> &desired_torque, imu::Vector<3> current_B) noexcept;
    };
} // namespace gnc_math
