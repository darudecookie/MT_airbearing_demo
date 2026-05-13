#include "Sensor/AnalogSensor/_AbstractAnalogSensor/_AbstractAnalogSensor.hpp"

#include <Arduino.h>

namespace sensor
{
    _AbstractAnalogSensor::_AbstractAnalogSensor(const uint8_t sensor_pin,
                                                 const double voltage_scaler) noexcept : _sensor_pin(sensor_pin),
                                                                                         _voltage_scaler(voltage_scaler)
    {
        pinMode(this->_sensor_pin, ANALOG);
    }

    double _AbstractAnalogSensor::read() const noexcept
    {

        return this->_voltage_scaler * (_AbstractAnalogSensor::ADC_MAX_VOLTS * static_cast<float>(analogRead(this->_sensor_pin)) / 4095.0);
    }
} // namespace sensor
