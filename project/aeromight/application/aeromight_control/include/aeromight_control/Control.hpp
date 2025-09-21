#pragma once

namespace aeromight_control
{

template <typename Logger>
class Control
{
public:
   explicit Control(Logger& logger)
       : m_logger{logger}
   {
      m_logger.enable();
   }

   void start()
   {
      m_enabled = true;
      m_logger.print("started control");
   }

   void stop()
   {
      m_enabled = false;
      m_logger.print("stopped control");
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
