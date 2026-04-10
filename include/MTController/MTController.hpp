#pragma once

#include <cstdint>
#include <cmath>

#include "SysStatemachine/SysStatemachine.hpp"

namespace mt_conn
{
    class MTController
    {
        const uint8_t _in1, _in2;
        const double _max_dutycycle;
        const bool _negative_dc;

        const sys_st::SysStateMachine &_st_machine;

    public:
        MTController(const uint8_t in1,
                     const uint8_t in2,
                     const double max_dutycycle,
                     const sys_st::SysStateMachine &st_machine,
                     const bool negative_dc = false) noexcept;

        void drive(const float dutycycle) const noexcept;
    };

} // namespace mt_conn
