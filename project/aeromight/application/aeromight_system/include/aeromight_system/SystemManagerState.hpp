#pragma once

namespace aeromight_system
{

enum class SystemManagerState
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

}   // namespace aeromight_system
