#pragma once

#include <cstdint>

#include "../_AbstractAnalogSensor/_AbstractAnalogSensor.hpp"

namespace sensor
{
    class CurrentSensor : public _AbstractAnalogSensor
    {
    public:
        CurrentSensor(const uint8_t sensor_pin,
                      const double volts_per_amp) : _AbstractAnalogSensor(sensor_pin, volts_per_amp) {}
    };
} // namespace sensor
