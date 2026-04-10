#pragma once

#include <imumaths.h>

namespace gnc_math
{
    namespace conn
    {
        class QuaternionPointingController
        {
            const double _kP, _kD;
            // note, we only use the negative of kP, so it makes sense to negate it at construction time instead of at every compute() call

            imu::Quaternion calc_error(const imu::Quaternion &setpoint,
                                                  const imu::Quaternion &current) const;

        public:
            QuaternionPointingController(const double kP, const double kD) : _kP(kP), _kD(kD) {}

            imu::Vector<3> compute_torque(const imu::Quaternion &setpoint,
                                                     const imu::Quaternion &current,
                                                     const imu::Vector<3> &omega) const;
        };
    } // namespace conn
} // namespace gnc_math

