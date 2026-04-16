#include "SysStatemachine/SysStatemachine.hpp"

namespace sys_st
{
    SystemTarget::SystemTarget(const possible_st st, imu::Vector<3> &target_data) noexcept : _st(st)
    {
        for (int i = 0; i < 3; i++)
        {
            this->_quat_vec_data[i] = static_cast<float>(target_data[i]);
        }
    }
    SystemTarget::SystemTarget(const possible_st st, imu::Quaternion &target_data) noexcept : _st(st)
    {
        this->_quat_vec_data = {static_cast<float>(target_data.w()),
                                static_cast<float>(target_data.x()),
                                static_cast<float>(target_data.y()),
                                static_cast<float>(target_data.z())};
    }

    possible_st SysStateMachine::transition_st(const possible_st target_st) noexcept
    {
    }
    possible_st SysStateMachine::update_target(const SystemTarget &new_target) noexcept
    {
        this->_desired_target = new_target;
        return this->transition_st(this->_desired_target.get_target_st());
    }

} // namespace sys_st
