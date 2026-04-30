#pragma once
#include <cstdint>

namespace sitl::hw
{

class Adc
{
public:
   explicit Adc(uint16_t initial_raw = 0)
       : m_raw{initial_raw}
   {
   }

   static constexpr void execute()
   {
      // no-op
   }

   void set_raw_value(const uint16_t raw)
   {
      m_raw = raw;
   }

   uint16_t get_raw_value() const
   {
      return m_raw;
   }

private:
   uint16_t m_raw{};
};

}   // namespace sitl::hw