#pragma once

#include "FlightManagerState.hpp"
#include "aeromight_boundaries/ControlSetpoints.hpp"
#include "aeromight_boundaries/FlightManagerData.hpp"
#include "aeromight_boundaries/HealthSummary.hpp"
#include "boundaries/SharedData.hpp"
#include "error/error_handler.hpp"
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
   using ControlSetpoints = boundaries::SharedData<aeromight_boundaries::ControlSetpoints>;
   using FlightSetpoints  = boundaries::SharedData<aeromight_boundaries::FlightSetpoints>;
   using RadioActuals     = boundaries::SharedData<aeromight_boundaries::RadioLinkStats>;

public:
   explicit FlightManagerStateHandler(QueueReceiver&         health_summary_queue_receiver,
                                      EstimationNotifier&    control_start_notifier,
                                      ControlSetpoints&      control_setpoints_storage,
                                      const FlightSetpoints& flight_setpoints_storage,
                                      const RadioActuals&    radio_link_actuals_storage,
                                      Logger&                logger,
                                      const float            stick_input_deadband_abs,
                                      const float            min_good_signal_rssi_dbm,
                                      const uint32_t         max_age_stale_data_ms,
                                      const uint32_t         min_state_debounce_duration_ms,
                                      const uint32_t         timeout_sensors_readiness_ms,
                                      const uint32_t         timeout_control_readiness_ms,
                                      const uint32_t         timeout_auto_land_ms)
       : m_health_summary_queue_receiver{health_summary_queue_receiver},
         m_control_start_notifier(control_start_notifier),
         m_control_setpoints_storage(control_setpoints_storage),
         m_flight_setpoints_storage(flight_setpoints_storage),
         m_radio_link_actuals_storage(radio_link_actuals_storage),
         m_logger{logger},
         m_stick_input_deadband_abs{stick_input_deadband_abs},
         m_min_good_signal_rssi_dbm{min_good_signal_rssi_dbm},
         m_max_age_stale_data_ms{max_age_stale_data_ms},
         m_min_state_debounce_duration_ms{min_state_debounce_duration_ms},
         m_timeout_sensors_readiness_ms{timeout_sensors_readiness_ms},
         m_timeout_control_readiness_ms{timeout_control_readiness_ms},
         m_timeout_auto_land_ms{timeout_auto_land_ms}
   {
   }

   void set_reference_time()
   {
      m_reference_time_ms = ClockSource::now_ms();
   }

   void get_time()
   {
      m_current_time_ms = ClockSource::now_ms();
   }

   void read_health_summary()
   {
      const auto summary = m_health_summary_queue_receiver.receive_latest();
      if (summary.has_value())
      {
         m_last_health_summary = summary.value();
      }
   }

   void read_radio_input()
   {
      m_last_flight_setpoints_storage   = m_flight_setpoints_storage.get_latest();
      m_last_radio_link_actuals_storage = m_radio_link_actuals_storage.get_latest();
   }

   void start_control()
   {
      m_control_start_notifier.notify();
      m_logger.print("started control");
   }

   void arm_control()
   {
      m_local_control_setpoints.mode  = aeromight_boundaries::ControlMode::idle;
      m_local_control_setpoints.armed = true;
      m_local_control_setpoints.kill  = false;
      m_control_setpoints_storage.update_latest(m_local_control_setpoints, m_current_time_ms);
   }

   void disarm_control()
   {
      m_local_control_setpoints.mode  = aeromight_boundaries::ControlMode::idle;
      m_local_control_setpoints.armed = false;
      m_local_control_setpoints.kill  = false;
      m_control_setpoints_storage.update_latest(m_local_control_setpoints, m_current_time_ms);
   }

   void kill_motors()
   {
      m_local_control_setpoints.mode  = aeromight_boundaries::ControlMode::disabled;
      m_local_control_setpoints.armed = false;
      m_local_control_setpoints.kill  = true;
      m_control_setpoints_storage.update_latest(m_local_control_setpoints, m_current_time_ms);
   }

   void publish_manual_setpoint()
   {
      m_local_control_setpoints.throttle = m_last_flight_setpoints_storage.data.input.throttle;
      m_local_control_setpoints.roll     = m_last_flight_setpoints_storage.data.input.roll;
      m_local_control_setpoints.pitch    = m_last_flight_setpoints_storage.data.input.pitch;
      m_local_control_setpoints.yaw      = m_last_flight_setpoints_storage.data.input.yaw;

      m_local_control_setpoints.mode  = aeromight_boundaries::ControlMode::manual_rate;
      m_local_control_setpoints.armed = true;
      m_local_control_setpoints.kill  = false;

      m_control_setpoints_storage.update_latest(m_local_control_setpoints, m_current_time_ms);
   }

   void publish_hover_setpoint()
   {
      m_local_control_setpoints.roll  = m_last_flight_setpoints_storage.data.input.roll;
      m_local_control_setpoints.pitch = m_last_flight_setpoints_storage.data.input.pitch;
      m_local_control_setpoints.yaw   = m_last_flight_setpoints_storage.data.input.yaw;

      m_local_control_setpoints.mode  = aeromight_boundaries::ControlMode::altitude_hold;
      m_local_control_setpoints.armed = true;
      m_local_control_setpoints.kill  = false;

      m_control_setpoints_storage.update_latest(m_local_control_setpoints, m_current_time_ms);
   }

   void publish_auto_land_setpoint()
   {
      m_local_control_setpoints.roll  = 0.0f;
      m_local_control_setpoints.pitch = 0.0f;
      m_local_control_setpoints.yaw   = m_last_flight_setpoints_storage.data.input.yaw;

      m_local_control_setpoints.mode  = aeromight_boundaries::ControlMode::auto_land;
      m_local_control_setpoints.armed = true;
      m_local_control_setpoints.kill  = false;

      m_control_setpoints_storage.update_latest(m_local_control_setpoints, m_current_time_ms);
   }

   void set_state(const FlightManagerState state)
   {
      m_state = state;

      switch (m_state)
      {
         case FlightManagerState::init:
            // do nothing
            break;

         case FlightManagerState::wait_sensors:
            m_logger.print("entered state->wait_sensors");
            break;

         case FlightManagerState::wait_control:
            m_logger.print("entered state->wait_control");
            break;

         case FlightManagerState::disarming:
            m_logger.print("entered state->disarming");
            break;

         case FlightManagerState::disarmed:
            m_logger.print("entered state->disarmed");
            break;

         case FlightManagerState::arming:
            m_logger.print("entered state->arming");
            break;

         case FlightManagerState::armed:
            m_logger.print("entered state->armed");
            break;

         case FlightManagerState::manual_mode:
            m_logger.print("entered state->manual_mode");
            break;

         case FlightManagerState::hover_mode:
            m_logger.print("entered state->hover_mode");
            break;

         case FlightManagerState::auto_land:
            m_logger.print("entered state->auto_land");
            break;

         case FlightManagerState::killed:
            m_logger.print("entered state->killed");
            break;

         case FlightManagerState::fault:
            m_logger.print("entered state->fault");
            break;

         default:
            error::stop_operation();
            break;
      }
   }

   FlightManagerState get_state() const
   {
      return m_state;
   }

   bool health_summary_received() const
   {
      return (m_last_health_summary.timestamp_ms > 0);
   }

   bool radio_input_received() const
   {
      return ((m_last_flight_setpoints_storage.timestamp_ms > 0) && (m_last_radio_link_actuals_storage.timestamp_ms > 0));
   }

   bool timeout_sensors_readiness() const
   {
      return ((m_current_time_ms - m_reference_time_ms) >= m_timeout_sensors_readiness_ms);
   }

   bool timeout_control_readiness() const
   {
      return ((m_current_time_ms - m_reference_time_ms) >= m_timeout_control_readiness_ms);
   }

   bool timeout_auto_land() const
   {
      return ((m_current_time_ms - m_reference_time_ms) >= m_timeout_auto_land_ms);
   }

   bool is_state_change_persistent() const
   {
      return ((m_current_time_ms - m_reference_time_ms) >= m_min_state_debounce_duration_ms);
   }

   bool sensors_ready() const
   {
      return m_last_health_summary.all_sensors_ready;
   }

   bool control_ready() const
   {
      return m_last_health_summary.estimation_ready;
   }

   bool arm() const
   {
      return (m_last_flight_setpoints_storage.data.state == aeromight_boundaries::FlightArmedState::arm);
   }

   bool disarm() const
   {
      return (m_last_flight_setpoints_storage.data.state == aeromight_boundaries::FlightArmedState::disarm);
   }

   bool kill() const
   {
      return m_last_flight_setpoints_storage.data.kill_switch_active;
   }

   bool manual_mode() const
   {
      return (m_last_flight_setpoints_storage.data.mode == aeromight_boundaries::FlightMode::stabilized_manual);
   }

   bool hover_mode() const
   {
      return (m_last_flight_setpoints_storage.data.mode == aeromight_boundaries::FlightMode::altitude_hold);
   }

   bool stale_health() const
   {
      return ((m_current_time_ms - m_last_health_summary.timestamp_ms) >= m_max_age_stale_data_ms);
   }

   bool stale_radio_input() const
   {
      if (((m_current_time_ms - m_last_flight_setpoints_storage.timestamp_ms) >= m_max_age_stale_data_ms) ||
          ((m_current_time_ms - m_last_radio_link_actuals_storage.timestamp_ms) >= m_max_age_stale_data_ms))
      {
         return true;
      }

      return false;
   }

   bool is_health_good() const
   {
      if (!stale_health())
      {
         return ((m_last_health_summary.flight_health != aeromight_boundaries::FlightHealthStatus::critical) &&
                 is_imu_operational() &&
                 is_estimation_operational() &&
                 is_control_operational());
      }

      return false;
   }

   bool is_radio_link_good() const
   {
      if (!stale_radio_input())
      {
         return (m_last_radio_link_actuals_storage.data.link_status_ok && (m_last_radio_link_actuals_storage.data.link_rssi_dbm >= m_min_good_signal_rssi_dbm));
      }

      return false;
   }

   // cppcheck-suppress functionStatic
   bool takeover_requested() const
   {
      return false;
   }

   // cppcheck-suppress functionStatic
   bool landing_complete() const
   {
      return false;
   }

private:
   bool is_imu_operational() const
   {
      return (m_last_health_summary.imu_health == aeromight_boundaries::SubsystemHealth::operational);
   }

   bool is_estimation_operational() const
   {
      return (m_last_health_summary.estimation_health == aeromight_boundaries::SubsystemHealth::operational);
   }

   bool is_control_operational() const
   {
      return (m_last_health_summary.control_health == aeromight_boundaries::SubsystemHealth::operational);
   }

   QueueReceiver&                         m_health_summary_queue_receiver;
   EstimationNotifier&                    m_control_start_notifier;
   ControlSetpoints&                      m_control_setpoints_storage;
   const FlightSetpoints&                 m_flight_setpoints_storage;
   const RadioActuals&                    m_radio_link_actuals_storage;
   Logger&                                m_logger;
   const float                            m_stick_input_deadband_abs;
   const float                            m_min_good_signal_rssi_dbm;
   const uint32_t                         m_max_age_stale_data_ms;
   const uint32_t                         m_min_state_debounce_duration_ms;
   const uint32_t                         m_timeout_sensors_readiness_ms;
   const uint32_t                         m_timeout_control_readiness_ms;
   const uint32_t                         m_timeout_auto_land_ms;
   aeromight_boundaries::ControlSetpoints m_local_control_setpoints{};
   aeromight_boundaries::HealthSummary    m_last_health_summary{};
   FlightSetpoints::Sample                m_last_flight_setpoints_storage{};
   RadioActuals::Sample                   m_last_radio_link_actuals_storage{};
   FlightManagerState                     m_state{FlightManagerState::init};
   uint32_t                               m_current_time_ms{0};
   uint32_t                               m_reference_time_ms{0};
};

}   // namespace aeromight_flight
