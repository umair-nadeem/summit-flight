#pragma once

namespace aeromight_boundaries
{

enum class FlightHealthStatus
{
   nominal,    // good
   degraded,   // non-critical failure (baro, gps, etc.)
   critical    // flight critical fault
};

}   // namespace aeromight_boundaries
