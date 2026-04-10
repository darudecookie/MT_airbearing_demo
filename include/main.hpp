#include <cstdint>
#include <array>

#include <Arduino.h>
#include <Adafruit_BNO055.h>

namespace io_params
{

    static constexpr std::array<std::array<uint8_t, 2>, 3> CURRENT_SENSOR_PINS = {std::array<uint8_t, 2>{0, 1},
                                                                                  std::array<uint8_t, 2>{0, 1},
                                                                                  std::array<uint8_t, 2>{0, 1}};
} // namespace io_params

namespace sys_params
{
    static constexpr std::array<float, 3> MT_CURRENT_LIM_A = {0.182, 0.182, 0.182},
                                          MT_VOLTAGE_LIM_V = {1e-5, 1e-5, 1e-5};
    static constexpr float SUPPLY_VOLTAGE_V = 1;

    static constexpr float MT_CURRENT_WATCHDOG_CHECK_FREQ_HZ = 200,
                           IMU_POLL_FREQ_HZ = 100,
                           MT_CONTROL_FREQ_HZ = 5,
                           COMMUNICATION_WRITE_FREQ_HZ = 200,
                           DEBUG_LED_FREQ_HZ = 10;

} // namespace sys_params

namespace io_objs
{
    Adafruit_BNO055 IMU = Adafruit_BNO055(55, 0x28, &Wire);

} // namespace sensor_obs

namespace loop_objs
{
    imu::Quaternion curr_attitude = {1, 0, 0, 0};
    imu::Vector<3> curr_B = {0, 0, 0},
                   curr_B_dot = {0, 0, 0};

    uint32_t last_MT_watchdog_check_time_ms = 0,
             last_IMU_check_time_ms = 0,
             last_MT_control_check_time_ms = 0;
}