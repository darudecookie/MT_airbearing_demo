/*

this code is a frankenstein of old rw and less old adcs stuff shamelessly copy-pasted
there is a lot of global state but its in namespaces so dont @ me
if something looks weird i probably wrote it drunk

*/

#include "main.hpp"

#include "GNCMath/ActuatorHandler/ActuatorHandler.hpp"

void setup()
{
    io_objs::comm_handler.report_error(io_objs::comm_handler.init_wifi(io_params::WIFI_NETWORK_NAME, io_params::WIFI_NETWORK_PASSWORD), millis());

    while (true)
    {
        io_objs::comm_handler.report_error(io_objs::comm_handler.attempt_client_conn(), millis());
        delay(500);
    }
    if (io_objs::BNO055_IMU.begin() == false)
    {
        io_objs::comm_handler.report_error(comm_handle::error_codes::imu_init_fail, millis());
    }
}

void loop()
{
    const uint32_t curr_time_ms = millis();

    // i dont want to deal with an rtos so were doing it the old fashioned way
    if ((curr_time_ms - loop_objs::last_MT_watchdog_check_time_ms) > static_cast<uint32_t>(1e3 / sys_params::MT_CURRENT_WATCHDOG_CHECK_FREQ_HZ))
    {
        task_funcs::check_watchdogs();
        loop_objs::last_MT_watchdog_check_time_ms = curr_time_ms;
    }

    const auto dt_ms = curr_time_ms - loop_objs::last_IMU_check_time_ms; // we use this a couple of times so gonna define it here
    if (dt_ms > static_cast<uint32_t>(1e3 / sys_params::IMU_POLL_FREQ_HZ))
    {
        task_funcs::update_IMU_data(dt_ms);
        loop_objs::last_IMU_check_time_ms = curr_time_ms;
    }

    if ((curr_time_ms - loop_objs::last_MT_control_check_time_ms) > static_cast<uint32_t>(1e3 / sys_params::MT_CONTROL_FREQ_HZ))
    {
        task_funcs::control_magnetorquers();
        loop_objs::last_MT_control_check_time_ms = curr_time_ms;
    }

    if ((curr_time_ms - loop_objs::last_comm_write_check_time_ms) > static_cast<uint32_t>(1e3 / sys_params::COMMUNICATION_WRITE_FREQ_HZ))
    {
        task_funcs::write_comm(curr_time_ms);
        loop_objs::last_comm_write_check_time_ms = curr_time_ms;
    }

    if ((curr_time_ms - loop_objs::last_LED_check_time_ms) > static_cast<uint32_t>(1e3 / sys_params::DEBUG_LED_FREQ_HZ))
    {
        task_funcs::update_LEDs();
        loop_objs::last_LED_check_time_ms = curr_time_ms;
    }
}

void task_funcs::check_watchdogs() noexcept {}
void task_funcs::update_IMU_data(const uint32_t dt_ms) noexcept
{
    for (auto &act : io_objs::magnetorquers)
    {
        act.set_lock(true);
    }

    curr_att::curr_attitude = io_objs::BNO055_IMU.getQuat();

    const auto new_ang_vel = io_objs::BNO055_IMU.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);
    curr_att::curr_ang_acc = util_funcs::vector_deriv(new_ang_vel, curr_att::curr_ang_vel, dt_ms);
    curr_att::curr_ang_vel = std::move(new_ang_vel);

    const auto new_B = io_objs::BNO055_IMU.getVector(Adafruit_BNO055::VECTOR_MAGNETOMETER);
    curr_att::curr_B_dot = util_funcs::vector_deriv(new_B, curr_att::curr_B, dt_ms);
    curr_att::curr_B = std::move(new_B);

    for (auto &act : io_objs::magnetorquers)
    {
        act.set_lock(false);
    }
}
void task_funcs::control_magnetorquers() noexcept
{
    const auto curr_st = loop_objs::sys_controller.get_state();
    if (curr_st == sys_st::possible_st::deactivated)
    {
        for (auto &actuator : io_objs::magnetorquers)
        {
            actuator.drive(0);
        }
    }
    else
    {
        imu::Vector<3> desired_torque(0, 0, 0);
        switch (loop_objs::sys_controller.get_state())
        {
        case sys_st::possible_st::direct_B_control:
            break;
        case sys_st::possible_st::b_dot_control:
            desired_torque = loop_objs::b_dot_conn.compute_torque(curr_att::curr_B_dot);
            break;
        case sys_st::possible_st::omega_slew_control:
            desired_torque = loop_objs::omega_slew_conn.compute_torque(loop_objs::sys_controller.get_target_vec(), curr_att::curr_ang_vel, curr_att::curr_ang_acc, micros());
            break;
        case sys_st::possible_st::quat_point_control:
            desired_torque = loop_objs::quat_point_conn.compute_torque(loop_objs::sys_controller.get_target_quat(), curr_att::curr_attitude, curr_att::curr_ang_vel);
            break;
        default:
            break;
        }
        const auto possible_torque = gnc_math::ActuatorHandler::possible_torque(desired_torque, curr_att::curr_B);

        for (uint8_t i = 0; i < io_objs::magnetorquers.size(); i++)
        {
            io_objs::magnetorquers[i].drive(possible_torque[i]);
        }
    }
}
void task_funcs::write_comm(const uint32_t current_time_ms) noexcept
{
    if (io_objs::comm_handler.should_update_telem())
    {
        io_objs::comm_handler.update_telem(current_time_ms, curr_att::curr_attitude, curr_att::curr_ang_vel, curr_att::curr_ang_acc,
                                           curr_att::curr_B, curr_att::curr_B_dot, io_objs::temp_sensors, io_objs::current_sensors);
    }

    if (io_objs::comm_handler.new_target_update())
    {
        loop_objs::sys_controller.update_target(io_objs::comm_handler.get_target_update());
    }
}
void task_funcs::update_LEDs() noexcept
{
}

imu::Vector<3> util_funcs::vector_deriv(const imu::Vector<3> &curr, const imu::Vector<3> &prev, const uint32_t dt_ms) noexcept
{
    return (curr - prev).scale(1e3 / static_cast<float>(dt_ms));
}
