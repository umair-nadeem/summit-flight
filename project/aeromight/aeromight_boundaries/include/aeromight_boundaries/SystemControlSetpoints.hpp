#pragma once

namespace aeromight_boundaries
{

enum class SystemArmedState : uint8_t
{
   disarm = 0,
   arm
};

struct SystemControlSetpoints
{
   SystemArmedState state{SystemArmedState::disarm};
};

}   // namespace aeromight_boundaries
