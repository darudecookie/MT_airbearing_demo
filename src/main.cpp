#include "main.hpp"

void setup()
{
}

void loop()
{
    const uint32_t curr_time_ms = millis();

    // i dont want to deal with an rtos so were doing it the old fashioned way
    if ((curr_time_ms - loop_objs::last_MT_watchdog_check_time_ms) > (1e3 / sys_params::MT_CURRENT_WATCHDOG_CHECK_FREQ_HZ))
    {
        
        loop_objs::last_MT_watchdog_check_time_ms = curr_time_ms;
    }
    if ((curr_time_ms - loop_objs::last_IMU_check_time_ms) > (1e3 / sys_params::IMU_POLL_FREQ_HZ))
    {

        loop_objs::last_IMU_check_time_ms = curr_time_ms;
    }
    if ((curr_time_ms - loop_objs::last_MT_control_check_time_ms) > (1e3 / sys_params::MT_CONTROL_FREQ_HZ))
    {

        loop_objs::last_MT_control_check_time_ms = curr_time_ms;
    }
}
