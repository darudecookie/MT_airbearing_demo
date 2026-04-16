#pragma once

#include <cstdint>
#include <array>

#include <imumaths.h>

#include "CommunicationHandler/comm_constants.hpp"

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

    class SystemTarget
    {
        possible_st _st;
        std::array<float, 4> _quat_vec_data = {}; // tried a union but it didnt get along with the 3rd part quat and vector classes

    public:
        SystemTarget(const possible_st st) noexcept : _st(st) {}
        SystemTarget(const possible_st st, imu::Vector<3> &target_data) noexcept;
        SystemTarget(const possible_st st, imu::Quaternion &target_data) noexcept;

        possible_st get_target_st() const noexcept { return this->_st; }
        imu::Vector<3> get_target_vec() const noexcept { return imu::Vector<3>(this->_quat_vec_data[0], this->_quat_vec_data[1], this->_quat_vec_data[2]); }
        imu::Quaternion get_target_quat() const noexcept { return imu::Quaternion(this->_quat_vec_data[0], this->_quat_vec_data[1], this->_quat_vec_data[2], this->_quat_vec_data[2]); }
    };

    class SysStateMachine
    {
        possible_st _curr_state;
        SystemTarget _desired_target;

    public:
        SysStateMachine(const possible_st init_state) noexcept : _curr_state(init_state), _desired_target(init_state) {}

        possible_st get_state() const noexcept { return this->_curr_state; }
        possible_st transition_st(const possible_st target_st) noexcept;

        possible_st update_target(const SystemTarget &new_target) noexcept;
        SystemTarget get_target() const noexcept { return this->_desired_target; }
        imu::Vector<3> get_target_vec() const noexcept { return this->_desired_target.get_target_vec(); }
        imu::Quaternion get_target_quat() const noexcept { return this->_desired_target.get_target_quat(); }
    };

} // namespace sys_st
