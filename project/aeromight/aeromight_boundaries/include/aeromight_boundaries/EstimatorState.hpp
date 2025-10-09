#pragma once

namespace aeromight_boundaries
{

enum class EstimatorState
{
   idle,
   get_pressure_reference,
   running,
   fault
};

}   // namespace aeromight_boundaries
