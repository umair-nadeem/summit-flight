#pragma once

namespace aeromight_flight
{

enum class FlightManagerState
{
   init,
   wait_sensors,
   wait_control,
   disarmed,
   armed,
   manual_mode,
   hover_mode,
   auto_land,
   killed,
   fault
};

}   // namespace aeromight_flight
