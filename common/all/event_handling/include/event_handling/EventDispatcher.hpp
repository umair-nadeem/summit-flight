#pragma once

#include <tuple>

#include "error/error_handler.hpp"
#include "interfaces/rtos/INotificationWaiter.hpp"
#include "types/types.hpp"

namespace event_handling
{

template <interfaces::rtos::INotificationWaiter NotificationWaiter, typename... Bindings>
class EventDispatcher
{
public:
   EventDispatcher(NotificationWaiter& notification_waiter,
                   const uint32_t      wait_timeout_ms,
                   Bindings... bindings)
       : m_notification_waiter(notification_waiter),
         m_wait_timeout_ms(wait_timeout_ms),
         m_bindings(std::move(bindings)...)
   {
      error::verify(m_wait_timeout_ms > 0u);
   }

   void execute()
   {
      const types::EventBitsType bits = m_notification_waiter.wait(m_wait_timeout_ms);

      if (bits == 0u)
      {
         return;
      }

      std::apply(
          [&](auto&... b)
          { (b.dispatch(bits), ...); },
          m_bindings);
   }

private:
   NotificationWaiter&     m_notification_waiter;
   uint32_t                m_wait_timeout_ms;
   std::tuple<Bindings...> m_bindings;
};

}   // namespace event_handling
