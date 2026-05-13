#pragma once

#include <cstdint>
#include <array>

namespace comm_handle
{
    static constexpr std::array<uint8_t, 2> MSG_START_WORD = {0b01010101, 0b10101010};

    enum class msg_codes : uint8_t
    {
        ping,
        set_telem_bitmask,
        error,

        set_deactivated,
        set_MT_dc_control,
        set_B_dot_control,
        set_omega_slew_control,
        set_quat_point_control,

        read_timestamp,
        read_sys_st,
        read_quat,
        read_ang_vel,
        read_ang_acc,
        read_MT_dc,
        read_B,
        read_B_dot,
        read_MT_temp,
        read_MT_current
    };

    static constexpr msg_codes LOWEST_TELEM_CODE = msg_codes::read_timestamp;
    static constexpr msg_codes HIGHEST_TELEM_CODE = msg_codes::read_MT_current;

    enum class error_codes : uint8_t
    {
        none = 0,

        wifi_init_fail = 10,
        websocket_client_connect_fail = 11,
        imu_init_fail = 12,

        mt_0_over_temp = 20,
        mt_1_over_temp = 21,
        mt_2_over_temp = 22,

        mt_0_overcurrent = 30,
        mt_1_overcurrent = 31,
        mt_2_overcurrent = 32,

        angular_velocity_exceeded = 40
    };
 
    static uint8_t msg_payload_len(const msg_codes code) noexcept
    {
        switch (code)
        {
        case msg_codes::ping:
            return 1;
        case msg_codes::set_telem_bitmask:
            return 4;
        case msg_codes::error:
            return 4;
        case msg_codes::set_deactivated:
            return 0;
        case msg_codes::set_MT_dc_control:
            return 3 * sizeof(float);
        case msg_codes::set_B_dot_control:
            return 0;
        case msg_codes::set_omega_slew_control:
            return 3 * sizeof(float);
        case msg_codes::set_quat_point_control:
            return 3 * sizeof(float);
        case msg_codes::read_timestamp:
            return 4;
        case msg_codes::read_sys_st:
            return 1;
        case msg_codes::read_quat:
            return 4 * sizeof(float);
        case msg_codes::read_ang_vel:
            return 3 * sizeof(float);
        case msg_codes::read_ang_acc:
            return 3 * sizeof(float);
        case msg_codes::read_MT_dc:
            return 3 * sizeof(float);
        case msg_codes::read_B:
            return 3 * sizeof(float);
        case msg_codes::read_B_dot:
            return 3 * sizeof(float);
        case msg_codes::read_MT_temp:
            return 3 * sizeof(float);
        case msg_codes::read_MT_current:
            return 3 * sizeof(float);
        default:
            return 0;
        }
    }
    static constexpr uint8_t telem_to_bitmask_index(const msg_codes code) noexcept
    {
        return static_cast<uint8_t>(code) - static_cast<uint8_t>(LOWEST_TELEM_CODE);
    }
    static constexpr msg_codes bitmask_index_to_telem(const uint8_t bitmask_i) noexcept
    {
        return static_cast<msg_codes>(bitmask_i + static_cast<uint8_t>(LOWEST_TELEM_CODE));
    }

    static constexpr bool should_report(const msg_codes code, const uint64_t bitmask) noexcept
    {
        return (bitmask & (uint64_t(1) << telem_to_bitmask_index(code)));
    }
} // namespace comm_handle
