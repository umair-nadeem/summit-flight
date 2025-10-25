#pragma once

#include "aeromight_boundaries/HealthSummary.hpp"
#include "interfaces/IClockSource.hpp"
#include "interfaces/rtos/INotifier.hpp"
#include "interfaces/rtos/IQueueReceiver.hpp"

namespace aeromight_flight
{

template <interfaces::rtos::IQueueReceiver<aeromight_boundaries::HealthSummary> QueueReceiver,
          interfaces::rtos::INotifier                                           EstimationNotifier,
          interfaces::IClockSource                                              ClockSource,
          typename Logger>
class FlightManagerStateHandler
{
public:
   explicit FlightManagerStateHandler(QueueReceiver&      health_summary_queue_receiver,
                                      EstimationNotifier& estimation_start_notifier,
                                      Logger&             logger,
                                      const uint32_t      max_wait_sensors_readiness_ms,
                                      const uint32_t      max_wait_estimation_control_readiness_ms)
       : m_health_summary_queue_receiver{health_summary_queue_receiver},
         m_estimation_start_notifier(estimation_start_notifier),
         m_logger{logger},
         m_max_wait_sensors_readiness_ms{max_wait_sensors_readiness_ms},
         m_max_wait_estimation_control_readiness_ms{max_wait_estimation_control_readiness_ms}
   {
   }

   void update_reference_time()
   {
      m_reference_time_ms = ClockSource::now_ms();
   }

   void read_health_summary()
   {
      const auto summary = m_health_summary_queue_receiver.receive_latest();
      if (summary.has_value())
      {
         m_last_health_summary = summary.value();
      }
   }

   // cppcheck-suppress functionStatic
   void start_estimation()
   {
      m_estimation_start_notifier.notify();
   }

   bool sensors_ready() const
   {
      return m_last_health_summary.all_sensors_ready;
   }

   bool timeout_sensors_readiness() const
   {
      return (ClockSource::now_ms() - m_reference_time_ms) >= m_max_wait_sensors_readiness_ms;
   }

private:
   QueueReceiver&                      m_health_summary_queue_receiver;
   EstimationNotifier&                 m_estimation_start_notifier;
   Logger&                             m_logger;
   const uint32_t                      m_max_wait_sensors_readiness_ms;
   const uint32_t                      m_max_wait_estimation_control_readiness_ms;
   aeromight_boundaries::HealthSummary m_last_health_summary{};
   uint32_t                            m_reference_time_ms{0};
};

}   // namespace aeromight_flight
