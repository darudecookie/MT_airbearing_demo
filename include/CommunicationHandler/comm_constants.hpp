#pragma once

#include <cstdint>
#include <array>

namespace comm_handle
{
    static constexpr std::array<uint8_t, 2> MSG_START_WORD = {0b01010101, 0b10101010};

    enum class msg_codes : uint8_t
    {
        ping = 0,
        timestamp = 1,
        set_telem_bitmask = 2,
        error = 3,
        UNUSED = 4,

        set_target_b_dot = 5,
        set_target_ang_vel = 6,
        set_target_quat = 7,

        read_timestamp = 8,
        read_quat = 9,
        read_ang_vel = 10,
        read_ang_acc = 11,
        read_B = 12,
        read_B_dot = 13,
        read_MT_temp = 14,
        read_MT_current = 15
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

    static constexpr std::array<uint8_t, 15> msg_code_to_length = {/* ping = 0 */ 0,
                                                                   /* timestamp = 1 */ sizeof(uint32_t),
                                                                   /* set_telem_bitmask = 2 */ sizeof(uint64_t),
                                                                   /* error = 3 */ 1,

                                                                   /* UNUSED = 4 */ 0,

                                                                   /* set_target_b_dot = 5 */ 0,
                                                                   /* set_target_ang_vel = 6 */ 0,
                                                                   /* set_target_quat = 7 */ 3 * sizeof(float),

                                                                   /* read_quat = 8 */ 4 * sizeof(float),
                                                                   /* read_ang_vel = 9 */ 3 * sizeof(float),
                                                                   /* read_ang_acc = 10 */ 3 * sizeof(float),
                                                                   /* read_B = 11 */ 3 * sizeof(float),
                                                                   /* read_B_dot = 12 */ 3 * sizeof(float),
                                                                   /* read_MT_temp = 13 */ 3 * sizeof(float),
                                                                   /* read_MT_current = 14 */ 3 * sizeof(float)};

    static constexpr uint8_t telem_to_bitmask_index(const msg_codes code) noexcept
    {
        return static_cast<uint8_t>(code) - static_cast<uint8_t>(LOWEST_TELEM_CODE);
    }
    static constexpr msg_codes bitmask_index_to_telem(const uint8_t bitmask_i) noexcept
    {
        return static_cast<msg_codes>(bitmask_i + static_cast<uint8_t>(LOWEST_TELEM_CODE));
    }

    static constexpr uint64_t generate_telem_bitmask(const bool report_timestamp, const bool report_quat, const bool report_ang_vel,
                                                     const bool report_ang_acc, const bool report_B, const bool report_B_dot,
                                                     const bool report_MT_temp, const bool report_MT_current) noexcept
    {
        uint64_t out = (static_cast<uint64_t>(report_timestamp ? uint64_t(1) : 0) << 0) |
                       (static_cast<uint64_t>(report_quat ? uint64_t(1) : 0) << 1) |
                       (static_cast<uint64_t>(report_ang_vel ? uint64_t(1) : 0) << 2) |
                       (static_cast<uint64_t>(report_ang_acc ? uint64_t(1) : 0) << 3) |
                       (static_cast<uint64_t>(report_B ? uint64_t(1) : 0) << 4) |
                       (static_cast<uint64_t>(report_B_dot ? uint64_t(1) : 0) << 5) |
                       (static_cast<uint64_t>(report_MT_temp ? uint64_t(1) : 0) << 6) |
                       (static_cast<uint64_t>(report_MT_current ? uint64_t(1) : 0) << 7);

        return out;
    }

    static constexpr bool should_report(const msg_codes code, const uint64_t bitmask) noexcept
    {
        return (bitmask & (uint64_t(1) << telem_to_bitmask_index(code)));
    }
} // namespace comm_handle
