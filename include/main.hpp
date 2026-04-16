#pragma once

#include <cstdint>
#include <array>

#include <Arduino.h>
#include <Adafruit_BNO055.h>
#include <imumaths.h>

#include "SysStatemachine/SysStatemachine.hpp"
#include "CommunicationHandler/CommunicationHandler.hpp"

#include "GNCMath/Controllers/BDotController/BDotController.hpp"
#include "GNCMath/Controllers/OmegaSlewController/OmegaSlewController.hpp"
#include "GNCMath/Controllers/QuaternionPointingController/QuaternionPointingController.hpp"

#include "MTController/MTController.hpp"
#include "Sensor/AnalogSensor/TempSensor/TempSensor.hpp"
#include "Sensor/AnalogSensor/CurrentSensor/CurrentSensor.hpp"

namespace io_params
{

    static const char WIFI_NETWORK_NAME[] = "HSL_magnetorquer_demo";
    static const char WIFI_NETWORK_PASSWORD[] = "fTF2Z#(;,xRD'sr4hB`%kG";
    static const uint8_t WIFI_SERVER_MAX_CLIENTS = 2;
    static constexpr std::array<uint8_t, 4> WIFE_SERVER_IP_ADDR = {10, 19, 111, 237};
    static constexpr uint32_t WIFI_SERVER_PORT_NUM = 10000;

    // MUST BE PWM PINS
    static constexpr std::array<std::array<uint8_t, 2>, NUM_MAGNETORQUERS> MT_CONTROL_PINS = {std::array<uint8_t, 2>{2, 4},
                                                                                              std::array<uint8_t, 2>{16, 17},
                                                                                              std::array<uint8_t, 2>{5, 18}};

    // MUST BE ANALOG PINS
    static constexpr double TEMP_SENS_VOLTS_PER_DEG = .010; // https://www.analog.com/media/en/technical-documentation/data-sheets/TMP35_36_37.pdf: 10mV / *C
    static constexpr std::array<uint8_t, NUM_MAGNETORQUERS> TEMP_SENSOR_PINS = {13, 12, 14};

    static constexpr double CURRENT_SENS_VOLTS_PER_AMP = 1; // https://www.adafruit.com/product/1164: 1V / A
    static constexpr std::array<uint8_t, NUM_MAGNETORQUERS> CURRENT_SENSOR_PINS = {27, 26, 25};

} // namespace io_params

namespace sys_params
{
    static constexpr std::array<float, NUM_MAGNETORQUERS> MT_CURRENT_LIM_A = {0.182, 0.182, 0.182},
                                                          MT_VOLTAGE_LIM_V = {12, 12, 12};
    static constexpr float SUPPLY_VOLTAGE_V = 9;

    static constexpr double B_DOT_CONN_kP = 1;

    static constexpr double SLEW_CONN_kP = 1,
                            SLEW_CONN_kI = 0,
                            SLEW_CONN_kD = 0;

    static constexpr double QUAT_POINT_CONN_kP = 1,
                            QUAT_POINT_CONN_kD = 0;

    static constexpr float MT_CURRENT_WATCHDOG_CHECK_FREQ_HZ = 200,
                           IMU_POLL_FREQ_HZ = 100,
                           MT_CONTROL_FREQ_HZ = 5,
                           COMMUNICATION_WRITE_FREQ_HZ = 10,
                           DEBUG_LED_FREQ_HZ = 10;

} // namespace sys_params

namespace curr_att
{

    static imu::Quaternion curr_attitude = {1, 0, 0, 0};
    static imu::Vector<NUM_MAGNETORQUERS> curr_ang_vel = {0, 0, 0},
                                          curr_ang_acc = {0, 0, 0},
                                          curr_B = {0, 0, 0},
                                          curr_B_dot = {0, 0, 0};

} // namespace curr_att

namespace loop_objs
{
    static sys_st::SysStateMachine sys_controller(sys_st::possible_st::deactivated);
    //static comm_handle::TargetUpdate system_target;

    static gnc_math::conn::BDotController b_dot_conn(sys_params::B_DOT_CONN_kP);
    static gnc_math::conn::OmegaSlewController omega_slew_conn(sys_params::SLEW_CONN_kP, sys_params::SLEW_CONN_kI, sys_params::SLEW_CONN_kD);
    static gnc_math::conn::QuaternionPointingController quat_point_conn(sys_params::QUAT_POINT_CONN_kP, sys_params::QUAT_POINT_CONN_kD);

    static uint32_t last_MT_watchdog_check_time_ms = 0,
                    last_IMU_check_time_ms = 0,
                    last_MT_control_check_time_ms = 0,
                    last_comm_write_check_time_ms = 0,
                    last_LED_check_time_ms = 0;
} // namespace loop_objs

namespace io_objs
{
    static comm_handle::CommunicationHandler comm_handler(io_params::WIFE_SERVER_IP_ADDR, io_params::WIFI_SERVER_PORT_NUM, io_params::WIFI_SERVER_MAX_CLIENTS);

    static Adafruit_BNO055 BNO055_IMU;

    static std::array<mt_conn::MTController, NUM_MAGNETORQUERS> magnetorquers = {mt_conn::MTController(io_params::MT_CONTROL_PINS[0][0],
                                                                                                       io_params::MT_CONTROL_PINS[0][1],
                                                                                                       sys_params::MT_VOLTAGE_LIM_V[0] / sys_params::SUPPLY_VOLTAGE_V,
                                                                                                       loop_objs::sys_controller),
                                                                                 mt_conn::MTController(io_params::MT_CONTROL_PINS[1][0],
                                                                                                       io_params::MT_CONTROL_PINS[1][1],
                                                                                                       sys_params::MT_VOLTAGE_LIM_V[0] / sys_params::SUPPLY_VOLTAGE_V,
                                                                                                       loop_objs::sys_controller),
                                                                                 mt_conn::MTController(io_params::MT_CONTROL_PINS[2][0],
                                                                                                       io_params::MT_CONTROL_PINS[2][1],
                                                                                                       sys_params::MT_VOLTAGE_LIM_V[0] / sys_params::SUPPLY_VOLTAGE_V,
                                                                                                       loop_objs::sys_controller)};

    static std::array<sensor::TempSensor, NUM_MAGNETORQUERS> temp_sensors = {sensor::TempSensor(io_params::TEMP_SENSOR_PINS[0], io_params::TEMP_SENS_VOLTS_PER_DEG),
                                                                             sensor::TempSensor(io_params::TEMP_SENSOR_PINS[1], io_params::TEMP_SENS_VOLTS_PER_DEG),
                                                                             sensor::TempSensor(io_params::TEMP_SENSOR_PINS[2], io_params::TEMP_SENS_VOLTS_PER_DEG)};

    static std::array<sensor::CurrentSensor, NUM_MAGNETORQUERS> current_sensors = {sensor::CurrentSensor(io_params::CURRENT_SENSOR_PINS[0], io_params::CURRENT_SENS_VOLTS_PER_AMP),
                                                                                   sensor::CurrentSensor(io_params::CURRENT_SENSOR_PINS[1], io_params::CURRENT_SENS_VOLTS_PER_AMP),
                                                                                   sensor::CurrentSensor(io_params::CURRENT_SENSOR_PINS[2], io_params::CURRENT_SENS_VOLTS_PER_AMP)};

} // namespace sensor_obs

namespace task_funcs
{
    void check_watchdogs() noexcept;
    void update_IMU_data(const uint32_t dt_ms) noexcept;
    void control_magnetorquers() noexcept;
    void write_comm(const uint32_t current_time_ms) noexcept;
    void update_LEDs() noexcept;
} // namespace task_funcs

namespace util_funcs
{
    imu::Vector<3> vector_deriv(const imu::Vector<3> &curr,
                                const imu::Vector<3> &prev,
                                const uint32_t dt_ms) noexcept;

} // namespace util_funcs
