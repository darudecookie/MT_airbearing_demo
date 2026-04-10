#include <GNCMath/Controllers/BDotController/BDotController.hpp>

#include <imumaths.h>

namespace gnc_math
{
    namespace conn
    {
        imu::Vector<3> BDotController::compute_torque(const imu::Vector<3> &B_dot) const
        {
            const imu::Vector<3> out(this->_negative_k * B_dot.x(),
                                                this->_negative_k * B_dot.y(),
                                                this->_negative_k * B_dot.z());
            return out;
        }
    }
    // namespace conn
} // namespace controls
