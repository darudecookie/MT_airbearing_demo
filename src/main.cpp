#include "main.hpp"

void setup()
{
    io_objs::BNO055_IMU.begin();
}

void loop()
{
    const uint32_t curr_time_ms = millis();

    // i dont want to deal with an rtos so were doing it the old fashioned way
    if ((curr_time_ms - loop_objs::last_MT_watchdog_check_time_ms) > static_cast<uint32_t>(1e3 / sys_params::MT_CURRENT_WATCHDOG_CHECK_FREQ_HZ))
    {

        loop_objs::last_MT_watchdog_check_time_ms = curr_time_ms;
    }

    const auto dt_ms = curr_time_ms - loop_objs::last_IMU_check_time_ms;
    if (dt_ms > static_cast<uint32_t>(1e3 / sys_params::IMU_POLL_FREQ_HZ))
    {
        curr_att::curr_attitude = io_objs::BNO055_IMU.getQuat();

        const auto new_ang_vel = io_objs::BNO055_IMU.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);
        curr_att::curr_ang_acc = util_funcs::vector_deriv(new_ang_vel, curr_att::curr_ang_vel, dt_ms);
        curr_att::curr_ang_vel = std::move(new_ang_vel);

        const auto new_B = io_objs::BNO055_IMU.getVector(Adafruit_BNO055::VECTOR_MAGNETOMETER);
        curr_att::curr_B_dot = util_funcs::vector_deriv(new_B, curr_att::curr_B, dt_ms);
        curr_att::curr_B = std::move(new_B);

        loop_objs::last_IMU_check_time_ms = curr_time_ms;
    }

    if ((curr_time_ms - loop_objs::last_MT_control_check_time_ms) > static_cast<uint32_t>(1e3 / sys_params::MT_CONTROL_FREQ_HZ))
    {
        const auto curr_st = loop_objs::sys_controller.get_state();
        if (curr_st == loop_objs::sys_controller.get_state())
        {
            for (int i = 0; i < 3; i++)
            {
                io_objs::magnetorquers[i].drive(0);
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
                desired_torque = loop_objs::omega_slew_conn.compute_torque(loop_objs::target_ang_vel, curr_att::curr_ang_vel, curr_att::curr_ang_acc, micros());
                break;
            case sys_st::possible_st::quat_point_control:
                desired_torque = loop_objs::quat_point_conn.compute_torque(loop_objs::target_attitude, curr_att::curr_attitude, curr_att::curr_ang_vel);
                break;
            default:
                break;
            }
            
        }
        loop_objs::last_MT_control_check_time_ms = curr_time_ms;
    }
}

imu::Vector<3> util_funcs::vector_deriv(const imu::Vector<3> &curr, const imu::Vector<3> &prev, const uint32_t dt_ms)
{
    return (curr - prev).scale(1e3 / static_cast<float>(dt_ms));
}