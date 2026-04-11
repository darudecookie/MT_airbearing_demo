
#pragma once

#include <cstdint>

namespace sensor
{
    class _AbstractAnalogSensor
    {
        const uint8_t _sensor_pin;
        const double _voltage_scaler;
        static constexpr double ADC_MAX_VOLTS = 3.3;

    public:
        _AbstractAnalogSensor(const uint8_t sensor_pin, const double voltage_scaler) noexcept;
        double read() const noexcept;
    };

} // namespace sensor
