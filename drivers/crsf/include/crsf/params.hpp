#pragma once

#include <cstdint>

namespace crsf
{

// Broadcast Frame:  [sync byte] [length] [type] [payload] [crc]
// Extended Header Frame: [sync byte] [length] [type] [dest address] [origin address] [payload] [crc]

// Frames with type lower than 0x27 are broadcast frames and have simple (short) header.

static constexpr std::size_t max_buffer_size = 64u;

static constexpr std::size_t rc_channel_count = 16u;

static constexpr uint8_t sync_byte = 0xc8;

static constexpr uint8_t num_bytes_sync_byte  = 1u;
static constexpr uint8_t num_bytes_frame_size = 1u;
static constexpr uint8_t num_bytes_frame_type = 1u;
static constexpr uint8_t num_bytes_frame_crc  = 1u;

static constexpr uint16_t rc_channel_value_min = 172u;
static constexpr uint16_t rc_channel_value_max = 1811u;

enum class DeviceAddress : uint8_t
{
   broadcast              = 0x00,
   cloud                  = 0x0e,
   usb_device             = 0x10,
   bluetooth_wifi         = 0x12,
   wifi_receiver          = 0x13,
   video_receiver         = 0x14,
   osd                    = 0x80,
   voltage_current_sensor = 0xc0,
   gps                    = 0xc2,
   tbs_blackbox           = 0xc4,
   flight_controller      = 0xc8,
   race_tag               = 0xcc,
   vtx                    = 0xce,
   remote_control         = 0xea,
   rc_receiver            = 0xec,
   rc_transmitter         = 0xee
};

enum class FrameType : uint8_t
{
   // broadcast frames
   gps                = 0x02,
   battery_sensor     = 0x08,
   airspeed           = 0x0a,
   heartbeat          = 0x0b,
   link_statistics    = 0x14,   // uplink - ground to UAV
   rc_channels_packed = 0x16,
   link_statistics_rx = 0x1c,
   link_statistics_tx = 0x1d,
   attitude           = 0x1e,
};

enum class PayloadSize : uint8_t
{
   gps                = 15u,
   battery            = 8u,
   airspeed           = 2u,
   link_statistics    = 10u,
   rc_channels_packed = 22u,
   link_statistics_rx = 5u,
   link_statistics_tx = 6u,
   attitude           = 6u,
};

static constexpr std::size_t min_valid_frame_size = num_bytes_sync_byte +
                                                    num_bytes_frame_size +
                                                    num_bytes_frame_type +
                                                    num_bytes_frame_crc;

}   // namespace crsf
