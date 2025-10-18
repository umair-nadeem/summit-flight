#pragma once

namespace aeromight_boundaries
{

enum class EstimatorState
{
   idle,
   get_reference_pressure,
   running,
   fault
};

}   // namespace aeromight_boundaries
