#include "MTController/MTController.hpp"

#include <cmath>

#include <Arduino.h>
#include "SysStateMachine/SysStateMachine.hpp"

namespace mt_conn
{
    MTController::MTController(const uint8_t in1,
                               const uint8_t in2,
                               const double max_dutycycle,
                               const sys_st::SysStateMachine &st_machine,
                               const bool negative_dc) noexcept : _in1(in1),
                                                                  _in2(in2),
                                                                  _max_dutycycle(std::abs(max_dutycycle)),
                                                                  _st_machine(st_machine),
                                                                  _negative_dc(negative_dc)
    {
        pinMode(this->_in1, OUTPUT);
        pinMode(this->_in2, OUTPUT);
        this->drive(0);
    }

    void MTController::drive(const float dutycycle) noexcept
    {
        this->_current_dc = dutycycle;

        if ((this->_st_machine.get_state() == sys_st::possible_st::deactivated) || dutycycle == 0)
        {
            digitalWrite(_in1, 1); // both pins high -> brake mode
            digitalWrite(_in2, 1);
            return;
        }

        if (this->_negative_dc)
        {
            this->_current_dc *= -1.0; // flip dc based on wiring
        }

        if (std::abs(dutycycle) > this->_max_dutycycle)
        {
            // clamp dc at signed max val
            this->_current_dc = (std::signbit(dutycycle) ? -1 : 1) * this->_max_dutycycle;
        }

        if (dutycycle > 0)
        {

            analogWrite(_in1, static_cast<uint8_t>(static_cast<float>(255) * dutycycle));
            analogWrite(_in2, 0);
        }
        else
        {

            analogWrite(_in1, 0);
            analogWrite(_in2, static_cast<uint8_t>(static_cast<float>(255) * -dutycycle));
        }
    }

    void MTController::set_lock(const bool is_locked) noexcept
    {
        this->_locked = is_locked;
        if (this->_locked)
        {
            this->drive(0);
        }
        else
        {
            this->drive(this->_current_dc);
        }
    }

} // namespace mt_conn
