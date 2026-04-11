#include "GNCMath/ActuatorHandler/ActuatorHandler.hpp"

namespace gnc_math
{
    imu::Vector<3> ActuatorHandler::possible_torque(const imu::Vector<3> &desired_torque, imu::Vector<3> current_B) noexcept
    {
        current_B.normalize();
        return desired_torque - current_B.scale(desired_torque.dot(current_B));
    }

} // namespace gnc_math
