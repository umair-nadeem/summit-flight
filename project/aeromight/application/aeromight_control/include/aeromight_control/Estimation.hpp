#pragma once

#include "boundaries/SharedData.hpp"

namespace aeromight_control
{

template <typename Logger>
class Estimation
{
public:
   explicit Estimation(Logger& logger)
       : m_logger{logger}
   {
      m_logger.enable();
   }

   void start()
   {
      m_enabled = true;
      m_logger.print("started estimation");
   }

   void stop()
   {
      m_enabled = false;
      m_logger.print("stopped estimation");
   }

   void execute() const
   {
      if (m_enabled)
      {
      }
   }

private:
   Logger& m_logger;
   bool    m_enabled{false};
};

}   // namespace aeromight_control
