#pragma once

#include <cstdint>
#include <array>

#include <imumaths.h>

namespace gnc_math
{
    namespace conn
    {
        class OmegaSlewController
        {

            const double _kP, _kI, _kD;

            imu::Vector<3> _prev_error;
            uint64_t _previous_call_us;

        public:
            OmegaSlewController(const double kP,
                                const double kI,
                                const double kD);

            imu::Vector<3> compute_torque(const imu::Vector<3> &omega_setpoint,
                                          const imu::Vector<3> &current_omega,
                                          const imu::Vector<3> &alpha,
                                          const uint64_t current_time_us);

            imu::Vector<3> get_error_term() { return this->_prev_error; }
            void reset_error_term() { this->_prev_error.scale(0); }
        };
    } // namespace conn
} // namespace gnc_math
