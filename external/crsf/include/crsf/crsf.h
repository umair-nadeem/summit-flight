/****************************************************************************
 *
 *	Copyright (c) 2018 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *	notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *	notice, this list of conditions and the following disclaimer in
 *	the documentation and/or other materials provided with the
 *	distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *	used to endorse or promote products derived from this software
 *	without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
 * @file crsf.h
 *
 * RC protocol definition for CSRF (TBS Crossfire).
 * It is an uninverted protocol at 420000 baudrate.
 *
 * RC channels come in at 150Hz.
 *
 * @author Beat Küng <beat-kueng@gmx.net>
 */

#pragma once

#include <array>
#include <cstdint>
#include <span>

namespace crsf
{

static constexpr std::size_t total_num_channels    = 16u;
static constexpr std::size_t crsf_frame_size_max   = 30u;   // the actual maximum length is 64, but we're only interested in RC channels and want to minimize buffer size
static constexpr std::size_t crsf_payload_size_max = (crsf_frame_size_max - 4u);

struct crsf_frame_header_t
{
   uint8_t device_address;   ///< @see crsf_address_t
   uint8_t length;           ///< length of crsf_frame_t (including CRC) minus sizeof(crsf_frame_header_t)
};

#pragma pack(push, 1)
struct crsf_frame_t
{
   crsf_frame_header_t header;
   uint8_t             type;                                  ///< @see crsf_frame_type_t
   uint8_t             payload[crsf_payload_size_max + 1u];   ///< payload data including 1 byte CRC at end
};
#pragma pack(pop)

/**
 * Parse the CRSF protocol and extract RC channel data.
 *
 * @param now current time
 * @param frame data to parse
 * @param len length of frame
 * @param values output channel values, each in range [1000, 2000]
 * @param num_values set to the number of parsed channels in values
 * @param max_channels maximum length of values
 * @return true if channels successfully decoded
 */
bool crsf_parse(std::span<const uint8_t> input_buffer, std::span<uint16_t> channels_out);

/**
 * Send telemetry battery information
 * @param uart_fd UART file descriptor
 * @param voltage Voltage [0.1V]
 * @param current Current [0.1A]
 * @param fuel drawn mAh
 * @param remaining battery remaining [%]
 * @return true on success
 */
bool crsf_send_telemetry_battery(int uart_fd, uint16_t voltage, uint16_t current, int32_t fuel, uint8_t remaining);

/**
 * Send telemetry GPS information
 * @param uart_fd UART file descriptor
 * @param latitude latitude [degree * 1e7]
 * @param longitude longitude [degree * 1e7]
 * @param groundspeed Ground speed [km/h * 10]
 * @param gps_heading GPS heading [degree * 100]
 * @param altitude Altitude [meters + 1000m offset]
 * @param num_satellites number of satellites used
 * @return true on success
 */
bool crsf_send_telemetry_gps(int uart_fd, int32_t latitude, int32_t longitude, uint16_t groundspeed,
                             uint16_t gps_heading, uint16_t altitude, uint8_t num_satellites);

/**
 * Send telemetry Attitude information
 * @param uart_fd UART file descriptor
 * @param pitch Pitch angle [rad * 1e4]
 * @param roll Roll angle [rad * 1e4]
 * @param yaw Yaw angle [rad * 1e4]
 * @return true on success
 */
bool crsf_send_telemetry_attitude(int uart_fd, int16_t pitch, int16_t roll, int16_t yaw);

/**
 * Send telemetry Flight Mode information
 * @param uart_fd UART file descriptor
 * @param flight_mode Flight Mode string (max length = 15)
 * @return true on success
 */
bool crsf_send_telemetry_flight_mode(int uart_fd, const char* flight_mode);

}   // namespace crsf
