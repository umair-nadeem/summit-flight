#pragma once

namespace aeromight_battery
{

struct PercentageConvertor16V8
{
   static constexpr uint8_t to_percentage(int32_t mv) noexcept
   {
      if (mv >= 16800)
         return 100u;
      if (mv >= 16400)
         return 90u;
      if (mv >= 16000)
         return 80u;
      if (mv >= 15600)
         return 70u;
      if (mv >= 15200)
         return 60u;
      if (mv >= 14800)
         return 50u;
      if (mv >= 14400)
         return 40u;
      if (mv >= 14000)
         return 30u;
      if (mv >= 13800)
         return 20u;
      if (mv >= 13600)
         return 10u;
      return 0;
   }
};

}   // namespace aeromight_battery
