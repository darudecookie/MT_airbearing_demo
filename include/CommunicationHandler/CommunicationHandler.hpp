#pragma once

#include <cstdint>

#include <imumaths.h>
#include <HardwareSerial.h>
#include <WiFiServer.h>
#include <WiFiClient.h>

#include "comm_constants.hpp"
#include "SysStatemachine/SysStatemachine.hpp"
#include "Sensor/AnalogSensor/TempSensor/TempSensor.hpp"
#include "Sensor/AnalogSensor/CurrentSensor/CurrentSensor.hpp"

namespace comm_handle
{
    struct TargetUpdate
    {
        sys_st::possible_st _target_type = sys_st::possible_st::deactivated;
        std::array<float, 4> _data = {};

        imu::Quaternion get_quat_data() const noexcept { return imu::Quaternion(this->_data[0], this->_data[1], this->_data[2], this->_data[3]); }
        imu::Vector<3> get_vector_data() const noexcept { return imu::Vector<3>(this->_data[0], this->_data[1], this->_data[2]); }
    };

    struct DataPackage
    {

        uint64_t _telem_bitmask = 0;

        uint32_t _timestamp = 0;

        imu::Quaternion _quat = {};
        imu::Vector<3> _ang_vel = {},
                       _ang_acc = {},
                       _B_val = {},
                       _B_dot = {};
        std::array<float, 3> _MT_temps = {},
                             _MT_currents = {};
    };

    namespace debug
    {
        static constexpr bool USE_DEBUG_UART_PORT = true;
        static HardwareSerial &DEBUG_UART_PORT = Serial;
    } // namespace debug

    class CommunicationHandler
    {
        WiFiServer _wifi_server;
        const uint16_t _port_num;
        WiFiClient _websocket_client;

        TargetUpdate _target_update = {};
        bool _new_target_update = false;

        DataPackage _telem_snapshot = {};
        uint64_t _sent_telem_bitmask = 0;

    public:
        CommunicationHandler(const std::array<uint8_t, 4> &address, const uint16_t port_num, const uint8_t max_clients, const uint64_t default_telem_bitmask = UINT64_MAX) noexcept;
        comm_handle::error_codes init_wifi(const char ssid[], const char password[]) noexcept;
        comm_handle::error_codes attempt_client_conn() noexcept;

        comm_handle::error_codes check_client_conn() noexcept
        {
            return (this->_websocket_client.connected() ? comm_handle::error_codes::none
                                                        : comm_handle::error_codes::websocket_client_connect_fail);
        }
        void report_error(const comm_handle::error_codes e, const uint32_t timestamp) noexcept;
        void exchange_data() noexcept;

        bool new_target_update() const noexcept { return this->_new_target_update; }
        TargetUpdate get_target_update() noexcept;

        bool should_update_telem() const noexcept { return (this->_sent_telem_bitmask == this->_telem_snapshot._telem_bitmask); }
        void update_telem(const uint32_t current_timestamp,
                          const imu::Quaternion &quat,
                          const imu::Vector<3> &ang_vel,
                          const imu::Vector<3> &ang_acc,
                          const imu::Vector<3> &B_field,
                          const imu::Vector<3> &B_dot,
                          const std::array<sensor::TempSensor, 3> &MT_temp,
                          const std::array<sensor::CurrentSensor, 3> &MT_current) noexcept;
    };
} // namespace comm_handle
