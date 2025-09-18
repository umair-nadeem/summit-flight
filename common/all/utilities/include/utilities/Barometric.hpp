#pragma once

#include <cmath>
#include <optional>

namespace utilities
{

class Barometric
{

   // h = h_0 + (t_0/l_0)((p/p_0)^(-r.l_0/g.m) - 1)
   // p_0 = static pressure (pressure at sea level) [Pa]
   // t_0 = standard temperature (temperature at sea level) [K]
   // l_0 = standard temperature lapse rate [K/m] = -0.0065 [K/m]
   // h = height about sea level [m]
   // h_0 = height at the bottom of atmospheric layer [m] -> 0
   // r = universal gas constant = 8.31432[N.m/mol.K]
   // g = gravitational acceleration constant = 9.80665
   // m = molar mass of Earth’s air = 0.0289644 [kg/mol]

public:
   static constexpr float h_0 = 0.0f;                                // m
   static constexpr float t_0 = 288.15f;                             // K
   static constexpr float l_0 = 0.0065f;                             // K/m
   static constexpr float r   = 8.31432f;                            // [N.m/mol.K]
   static constexpr float g   = 9.80665f;                            // m/s^2
   static constexpr float m   = 0.0289644f;                          // [kg/mol]

   static constexpr float pressure_exponent = (r * l_0) / (g * m);   // ~0.1902631
   static constexpr float altitude_constant = t_0 / l_0;             // ~44330.76923

   static std::optional<float> convert_pressure_to_altitude(const float pressure_pa, const float static_pressure_at_sea_level = 101325.0f)
   {
      if (!(pressure_pa > 0.0f) && !(static_pressure_at_sea_level > 0.0f))
      {
         return std::nullopt;
      }

      const float ratio  = pressure_pa / static_pressure_at_sea_level;
      const float factor = std::pow(ratio, pressure_exponent);
      const float h      = h_0 + (altitude_constant * (1.0f - factor));
      return h;
   }
};
}   // namespace utilities
