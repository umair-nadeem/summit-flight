#pragma once

namespace aeromight_system
{

enum class SystemManagerState
{
   init,
   wait_sensors,
   wait_control,
   imu_calibration,
   disarming,
   disarmed,
   arming,
   armed,
   fault
};

}   // namespace aeromight_system
