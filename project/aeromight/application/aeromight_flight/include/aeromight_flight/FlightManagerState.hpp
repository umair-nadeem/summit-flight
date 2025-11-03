#pragma once

namespace aeromight_flight
{

enum class FlightManagerState
{
   init,
   wait_sensors,
   wait_control,
   disarming,
   disarmed,
   arming,
   armed,
   manual_mode,
   hover_mode,
   auto_land,
   killed,
   fault
};

}   // namespace aeromight_flight
