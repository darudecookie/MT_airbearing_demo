#pragma once

#include <cstdint>
#include <array>

namespace sys_st
{
    enum class possible_st : uint8_t
    {
        deactivated,
        direct_B_control,
        b_dot_control,
        omega_slew_control,
        quat_point_control
    };

    class SysStateMachine
    {
        possible_st _curr_state;

    public:
        SysStateMachine(const possible_st init_state) noexcept : _curr_state(init_state) {}
        SysStateMachine() = delete;

        possible_st transition_st(const possible_st target_st) noexcept;

        possible_st get_state() const noexcept { return this->_curr_state; }
    };

} // namespace sys_st
