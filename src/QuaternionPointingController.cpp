#include <GNCMath/Controllers/QuaternionPointingController/QuaternionPointingController.hpp>

namespace gnc_math
{
    namespace conn
    {
        imu::Quaternion QuaternionPointingController::calc_error(const imu::Quaternion &setpoint,
                                                                 const imu::Quaternion &current) const
        {
            return setpoint * current.conjugate();
        }

        // i feel pretty bad about this whole function, it has a lot of indirection and hidden function calls and all of it is floating point math.
        // i don't know enough to optimize and i don't want to prematurely optimize but we probably should do some benchmarks to see how bad it is
        imu::Vector<3> QuaternionPointingController::compute_torque(const imu::Quaternion &setpoint,
                                                                    const imu::Quaternion &current,
                                                                    const imu::Vector<3> &omega) const
        {
            const imu::Quaternion error = calc_error(setpoint, current);
            const imu::Vector<3> xyz(error.x(), error.y(), error.z());
            return ((xyz * this->_kP) + (omega * this->_kD)).invert();
        }
    } // namespace conn
} // namespace gnc_math
