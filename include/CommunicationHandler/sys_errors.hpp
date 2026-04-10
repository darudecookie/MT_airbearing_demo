#pragma once

#include <cstdint>

namespace comm_handler
{
    enum class sys_errors : uint8_t
    {
        none,
        mt_0_overcurrent,
        mt_1_overcurrent,
        mt_2_overcurrent,
        angular_velocity_exceeded
    };

} // namespace comm_handler
