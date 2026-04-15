#pragma once

#include <cstdint>

#include "../_AbstractAnalogSensor/_AbstractAnalogSensor.hpp"

namespace sensor
{
    class TempSensor : public _AbstractAnalogSensor
    {

    public:
        TempSensor(const uint8_t sensor_pin,
                   const double volts_per_deg) : _AbstractAnalogSensor(sensor_pin, volts_per_deg) {}
    };
} // namespace sensor
