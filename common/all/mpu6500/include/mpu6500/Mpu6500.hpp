#pragma once

#include "Events.hpp"
#include "MainSM.hpp"
#include "Mpu6500StateHandler.hpp"

namespace mpu6500
{

template <interfaces::IClockSource ClockSource, typename SpiMaster, typename Logger>
class Mpu6500
{
   using ImuData   = ::boundaries::SharedData<imu_sensor::ImuData>;
   using ImuHealth = ::boundaries::SharedData<imu_sensor::ImuHealth>;

public:
   explicit Mpu6500(ImuData&          imu_data_storage,
                    ImuHealth&        imu_health_storage,
                    SpiMaster&        spi_master,
                    Logger&           logger,
                    const uint8_t     read_failures_limit,
                    const std::size_t execution_period_ms,
                    const std::size_t receive_wait_timeout_ms,
                    const uint8_t     sample_rate_divider,
                    const uint8_t     dlpf_config,
                    const uint8_t     gyro_full_scale,
                    const uint8_t     accel_full_scale,
                    const uint8_t     accel_a_dlpf_config,
                    const float       gyro_range_plausibility_margin_radps,
                    const float       accel_range_plausibility_margin_mps2,
                    const uint16_t    num_samples_self_test,
                    const float       gyro_tolerance_radps,
                    const float       accel_tolerance_mps2)
       : m_state_handler{imu_data_storage,
                         imu_health_storage,
                         spi_master,
                         logger,
                         read_failures_limit,
                         execution_period_ms,
                         receive_wait_timeout_ms,
                         sample_rate_divider,
                         dlpf_config,
                         gyro_full_scale,
                         accel_full_scale,
                         accel_a_dlpf_config,
                         gyro_range_plausibility_margin_radps,
                         accel_range_plausibility_margin_mps2,
                         num_samples_self_test,
                         gyro_tolerance_radps,
                         accel_tolerance_mps2}
   {
   }

   void execute()
   {
      m_state_machine.process_event(EventTick{});
   }

   void start()
   {
      m_state_machine.process_event(EventStart{});
   }

   void stop()
   {
      m_state_machine.process_event(EventStop{});
   }

   void notify_receive_complete()
   {
      m_state_machine.process_event(EventReceiveDone{});
   }

   imu_sensor::ImuSensorState get_state() const
   {
      return m_state_handler.get_state();
   }

   imu_sensor::ErrorBits get_error() const
   {
      return m_state_handler.get_error();
   }

private:
   using StateHandler    = Mpu6500StateHandler<ClockSource, SpiMaster, Logger>;
   using StateMachineDef = MainStateMachine<StateHandler>;

   StateHandler                    m_state_handler;
   boost::sml::sm<StateMachineDef> m_state_machine{m_state_handler};
};

}   // namespace mpu6500
