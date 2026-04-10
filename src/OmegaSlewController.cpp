#include <GNCMath/Controllers/OmegaSlewController/OmegaSlewController.hpp>

namespace gnc_math
{
    namespace conn
    {
        OmegaSlewController::OmegaSlewController(const double kP,
                                                 const double kI,
                                                 const double kD) : _kP(kP),
                                                                    _kI(kI),
                                                                    _kD(kD),
                                                                    _prev_error(0, 0, 0) //,
                                                                                         //                                                                    _previous_call_us{431}
        {
            this->_previous_call_us = 0;
        }

        imu::Vector<3> OmegaSlewController::compute_torque(const imu::Vector<3> &omega_setpoint,
                                                                      const imu::Vector<3> &current_omega,
                                                                      const imu::Vector<3> &alpha,
                                                                      const uint64_t current_time_us)
        {
            const imu::Vector<3> omega_error = omega_setpoint - current_omega;

            // P term = error * kP
            imu::Vector<3> out_u = omega_error.scale(this->_kP);

            // I term = (((prev_error + current_error) / 2) * time_step) * kI
            out_u = out_u + (this->_prev_error + omega_error).scale(0.5 * (static_cast<double>(current_time_us - this->_previous_call_us) / 1e6) * this->_kI);

            this->_previous_call_us = current_time_us;
            this->_prev_error = omega_error;

            // D term = - alpha * KD
            out_u = out_u - alpha.scale(this->_kD);

            return out_u;
        }
    } // namespace controls
} // namespace gnc_math
