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
   fault
};

}   // namespace aeromight_flight
