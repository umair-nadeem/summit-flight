#pragma once

#include <gmock/gmock.h>

#include "crsf/CrsfBattery.hpp"
#include "crsf/CrsfPacket.hpp"

namespace mocks::common
{

class Crsf
{
public:
   static bool parse_buffer(std::span<const uint8_t> buffer, crsf::CrsfPacket& packet)
   {
      buffer_to_parse = buffer;
      packet          = crsf_packet;
      return parse_result;
   }

   static uint32_t serialize_battery_telemetry(const crsf::CrsfBattery& packet, std::span<uint8_t> out)
   {
      battery_telemetry_packet = packet;
      telemetry_out_buffer     = out;
      return telemetry_bytes_written;
   }

   static void reset()
   {
      crsf_packet              = crsf::CrsfPacket{};
      battery_telemetry_packet = crsf::CrsfBattery{};
      telemetry_out_buffer     = std::span<uint8_t>{};
      buffer_to_parse          = std::span<const uint8_t>{};
      telemetry_bytes_written  = 0;
      parse_result             = false;
   }

   inline static crsf::CrsfPacket         crsf_packet{};
   inline static crsf::CrsfBattery        battery_telemetry_packet{};
   inline static std::span<const uint8_t> buffer_to_parse{};
   inline static std::span<uint8_t>       telemetry_out_buffer{};
   inline static uint32_t                 telemetry_bytes_written{};
   inline static bool                     parse_result{};
};

}   // namespace mocks::common
