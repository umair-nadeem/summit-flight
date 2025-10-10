#include "utilities/Barometric.hpp"

namespace utilities
{

float Barometric::m_pressure_at_sea_level_pa = 101325.0f;

std::optional<float> Barometric::convert_pressure_to_altitude(const float pressure_pa)
{
   if (pressure_pa <= 10.0f || m_pressure_at_sea_level_pa <= 10.0f)
   {
      return std::nullopt;
   }

   const float ratio  = pressure_pa / m_pressure_at_sea_level_pa;
   const float factor = powf(ratio, pressure_exponent);
   const float h      = h_0 + (altitude_constant * (1.0f - factor));
   return h;
}

void Barometric::set_pressure_reference(const float pressure_at_sea_level_pa)
{
   if (pressure_at_sea_level_pa > 100.0f)
   {
      m_pressure_at_sea_level_pa = pressure_at_sea_level_pa;
   }
}

}   // namespace utilities
