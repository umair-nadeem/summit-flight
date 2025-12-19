#pragma once

namespace aeromight_boundaries
{

enum class ControlState
{
   inactive,
   disarmed,
   armed_on_ground,
   airborne,
   emergency_kill
};

}   // namespace aeromight_boundaries
