#pragma once

namespace aeromight_link
{

// Tx channel mapping from Tx is at offset of 1 e.g. roll channel is 1 and so on...
static constexpr uint8_t rc_channel_roll     = 0;
static constexpr uint8_t rc_channel_pitch    = 1u;
static constexpr uint8_t rc_channel_throttle = 2u;
static constexpr uint8_t rc_channel_yaw      = 3u;

static constexpr uint8_t rc_channel_arm_state   = 5u;
static constexpr uint8_t rc_channel_flight_mode = 6u;
static constexpr uint8_t rc_channel_kill_switch = 7u;

}   // namespace aeromight_link
