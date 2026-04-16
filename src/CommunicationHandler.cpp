#ifndef COMMUNICATION_HANDLER_HPP_
#define COMMUNICATION_HANDLER_HPP_

#include "CommunicationHandler/CommunicationHandler.hpp"

#include <typeinfo>
#include <type_traits>

#include <WiFi.h>
#include <esp_wifi.h>

namespace comm_handle
{
    CommunicationHandler::CommunicationHandler(const std::array<uint8_t, 4> &address,
                                               const uint16_t port_num,
                                               const uint8_t max_clients,
                                               const uint64_t default_telem_bitmask) noexcept : _wifi_server(IPAddress(address.data()), port_num, max_clients),
                                                                                                _port_num(port_num),
                                                                                                _target_update(sys_st::possible_st::deactivated)
    {
        this->_telem_snapshot._telem_bitmask = default_telem_bitmask;
    }
    comm_handle::error_codes CommunicationHandler::init_wifi(const char ssid[], const char password[]) noexcept
    {
        // TODO: add wifi error checking
        WiFi.mode(WIFI_AP);
        WiFi.softAP(ssid, password);

        this->_wifi_server.begin(this->_port_num);

        return comm_handle::error_codes::none;
    }
    comm_handle::error_codes CommunicationHandler::attempt_client_conn() noexcept
    {

        if (this->_websocket_client.connected())
        {
            return comm_handle::error_codes::none;
        }

        this->_websocket_client = this->_wifi_server.accept();

        if (this->_websocket_client.connected())
        {
            return comm_handle::error_codes::none;
        }

        return comm_handle::error_codes::websocket_client_connect_fail;
    }

    void CommunicationHandler::report_error(const comm_handle::error_codes e, const uint32_t timestamp) noexcept
    {
    }
    void CommunicationHandler::exchange_data() noexcept
    {
        if (!this->should_update_telem())
        {

            msg_codes to_write_code = msg_codes::ping;

            const uint64_t unwritten = this->_telem_snapshot._telem_bitmask - this->_sent_telem_bitmask;
            for (uint8_t i = 0; i < static_cast<uint8_t>(HIGHEST_TELEM_CODE) - static_cast<uint8_t>(LOWEST_TELEM_CODE); i++)
            {
                if (unwritten & (uint64_t(1) << i))
                {
                    to_write_code = bitmask_index_to_telem(i);
                }
            }

            this->_websocket_client.write(MSG_START_WORD.data(), MSG_START_WORD.size());
            this->_websocket_client.write(static_cast<uint8_t>(to_write_code));
            switch (to_write_code)
            {
            case msg_codes::read_timestamp:
                this->_websocket_client.write(reinterpret_cast<uint8_t *>(this->_telem_snapshot._timestamp), sizeof(this->_telem_snapshot._timestamp));
                break;
            case msg_codes::read_quat:
            {
                const std::array<float, 4> quat_data = {static_cast<float>(this->_telem_snapshot._quat.w()),
                                                        static_cast<float>(this->_telem_snapshot._quat.x()),
                                                        static_cast<float>(this->_telem_snapshot._quat.y()),
                                                        static_cast<float>(this->_telem_snapshot._quat.z())};
                this->_websocket_client.write(reinterpret_cast<const uint8_t *>(quat_data.data()), sizeof(quat_data));
            }
            break;
            case msg_codes::read_ang_vel:
                this->_websocket_client.write(reinterpret_cast<uint8_t *>(&this->_telem_snapshot._ang_vel[0]), sizeof(this->_telem_snapshot._ang_vel));
                break;
            case msg_codes::read_ang_acc:
                this->_websocket_client.write(reinterpret_cast<const uint8_t *>(&this->_telem_snapshot._ang_acc[0]), sizeof(this->_telem_snapshot._ang_acc));
                break;
            case msg_codes::read_B:
                this->_websocket_client.write(reinterpret_cast<uint8_t *>(&this->_telem_snapshot._B_val[0]), sizeof(this->_telem_snapshot._B_val));
                break;
            case msg_codes::read_B_dot:
                this->_websocket_client.write(reinterpret_cast<uint8_t *>(&this->_telem_snapshot._B_dot[0]), sizeof(this->_telem_snapshot._B_dot));
                break;
            case msg_codes::read_MT_temp:
                this->_websocket_client.write(reinterpret_cast<uint8_t *>(this->_telem_snapshot._MT_temps.data()), sizeof(this->_telem_snapshot._MT_temps));
                break;
            case msg_codes::read_MT_current:
                this->_websocket_client.write(reinterpret_cast<uint8_t *>(this->_telem_snapshot._MT_currents.data()), sizeof(this->_telem_snapshot._MT_currents));
                break;
            default:
                break;
            }
        }
    }

sys_st::    SystemTarget CommunicationHandler::get_target_update() noexcept
    {
        this->_new_target_update = false;
        return this->_target_update;
    }

    void CommunicationHandler::update_telem(const uint32_t current_timestamp,
                                            const imu::Quaternion &quat,
                                            const imu::Vector<3> &ang_vel,
                                            const imu::Vector<3> &ang_acc,
                                            const imu::Vector<3> &B_field,
                                            const imu::Vector<3> &B_dot,
                                            const std::array<sensor::TempSensor, 3> &MT_temp,
                                            const std::array<sensor::CurrentSensor, 3> &MT_current) noexcept
    {

        if (should_report(msg_codes::read_quat, this->_telem_snapshot._telem_bitmask))
        {
            this->_telem_snapshot._quat = quat;
        }
        if (should_report(msg_codes::read_ang_vel, this->_telem_snapshot._telem_bitmask))
        {
            this->_telem_snapshot._ang_vel = ang_vel;
        }
        if (should_report(msg_codes::read_ang_acc, this->_telem_snapshot._telem_bitmask))
        {
            this->_telem_snapshot._ang_acc = ang_acc;
        }
        if (should_report(msg_codes::read_B, this->_telem_snapshot._telem_bitmask))
        {
            this->_telem_snapshot._B_val = B_field;
        }
        if (should_report(msg_codes::read_B_dot, this->_telem_snapshot._telem_bitmask))
        {
            this->_telem_snapshot._B_dot = B_dot;
        }
        if (should_report(msg_codes::read_MT_temp, this->_telem_snapshot._telem_bitmask))
        {
            for (uint8_t i = 0; i < MT_temp.size(); i++)
            {
                this->_telem_snapshot._MT_temps[i] = MT_temp[i].read();
            }
        }
        if (should_report(msg_codes::read_MT_current, this->_telem_snapshot._telem_bitmask))
        {
            for (uint8_t i = 0; i < MT_current.size(); i++)
            {
                this->_telem_snapshot._MT_currents[i] = MT_current[i].read();
            }
        }
        this->_sent_telem_bitmask = 0;
    }
} // namespace comm_handle

#endif // COMMUNICATION_HANDLER_HPP_