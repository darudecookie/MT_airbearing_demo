#pragma once

#include <cstdint>
#include <array>

#include <Arduino.h>
#include <Adafruit_BNO055.h>

#include "SysStatemachine/SysStatemachine.hpp"
#include "GNCMath/Controllers/BDotController/BDotController.hpp"
#include "GNCMath/Controllers/OmegaSlewController/OmegaSlewController.hpp"
#include "GNCMath/Controllers/QuaternionPointingController/QuaternionPointingController.hpp"
#include "GNCMath/ActuatorHandler/ActuatorHandler.hpp"
#include "MTController/MTController.hpp"

namespace io_params
{

    static const char WiFi_network_name[] = "HSL_magnetorquer_demo";
    static const char WiFi_password[] = "fTF2Z#(;,xRD'sr4hB`%kG";
    static constexpr std::array<uint32_t, 3> PC_IP_ADDRESS[] = {10, 19, 111, 237};
    static constexpr uint32_t PC_PORT_NUM = 10000;

    // MUST BE PWM PINS
    static constexpr std::array<std::array<uint8_t, 2>, 3> MT_CONTROL_PINS = {std::array<uint8_t, 2>{0, 1},
                                                                              std::array<uint8_t, 2>{0, 1},
                                                                              std::array<uint8_t, 2>{0, 1}};

} // namespace io_params

namespace sys_params
{
    static constexpr std::array<float, 3> MT_CURRENT_LIM_A = {0.182, 0.182, 0.182},
                                          MT_VOLTAGE_LIM_V = {1e-5, 1e-5, 1e-5};
    static constexpr float SUPPLY_VOLTAGE_V = 1;

    static constexpr double B_DOT_CONN_kP = 1,
                            SLEW_CONN_kP = 1,
                            SLEW_CONN_kI = 0,
                            SLEW_CONN_kD = 0,
                            QUAT_POINT_CONN_kP = 1,
                            QUAT_POINT_CONN_kD = 1;

    static constexpr float MT_CURRENT_WATCHDOG_CHECK_FREQ_HZ = 200,
                           IMU_POLL_FREQ_HZ = 100,
                           MT_CONTROL_FREQ_HZ = 5,
                           COMMUNICATION_WRITE_FREQ_HZ = 200,
                           DEBUG_LED_FREQ_HZ = 10;

} // namespace sys_params

namespace curr_att
{

    static imu::Quaternion curr_attitude = {1, 0, 0, 0};
    static imu::Vector<3> curr_ang_vel = {0, 0, 0},
                          curr_ang_acc = {0, 0, 0},
                          curr_B = {0, 0, 0},
                          curr_B_dot = {0, 0, 0};

} // namespace curr_att

namespace loop_objs
{
    static imu::Quaternion target_attitude = {1, 0, 0, 0};
    static imu::Vector<3> target_ang_vel = {0, 0, 0};

    static sys_st::SysStateMachine sys_controller(sys_st::possible_st::deactivated);

    static gnc_math::conn::BDotController b_dot_conn(sys_params::B_DOT_CONN_kP);
    static gnc_math::conn::OmegaSlewController omega_slew_conn(sys_params::SLEW_CONN_kP, sys_params::SLEW_CONN_kI, sys_params::SLEW_CONN_kD);
    static gnc_math::conn::QuaternionPointingController quat_point_conn(sys_params::QUAT_POINT_CONN_kP, sys_params::QUAT_POINT_CONN_kD);

    static uint32_t last_MT_watchdog_check_time_ms = 0,
                    last_IMU_check_time_ms = 0,
                    last_MT_control_check_time_ms = 0;
} // namespace loop_objs

namespace io_objs
{
    static Adafruit_BNO055 BNO055_IMU;

    static std::array<mt_conn::MTController, 3> magnetorquers = {mt_conn::MTController(io_params::MT_CONTROL_PINS[0][0],
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

} // namespace sensor_obs

namespace util_funcs
{
    imu::Vector<3> vector_deriv(const imu::Vector<3> &curr,
                                const imu::Vector<3> &prev,
                                const uint32_t dt_ms);
} // namespace util_funcs
