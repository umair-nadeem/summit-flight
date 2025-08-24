#pragma once

#include "Events.hpp"
#include "MainSM.hpp"
#include "Mpu6500StateHandler.hpp"

namespace mpu6500
{

template <interfaces::IClockSource ClockSource, typename SpiMaster>
class Mpu6500
{
   using ImuData   = ::boundaries::SharedData<imu_sensor::ImuData>;
   using ImuHealth = ::boundaries::SharedData<imu_sensor::ImuHealth>;

public:
   explicit Mpu6500(ImuData&          imu_data_storage,
                    ImuHealth&        imu_health_storage,
                    SpiMaster&        spi_master,
                    const uint8_t     read_failures_limit,
                    const std::size_t execution_period_ms,
                    const std::size_t receive_wait_timeout_ms,
                    const uint8_t     sample_rate_divider,
                    const uint8_t     dlpf_config,
                    const uint8_t     gyro_full_scale,
                    const uint8_t     accel_full_scale,
                    const uint8_t     accel_a_dlpf_config,
                    const float       gyro_range_plausibility_margin_radps,
                    const float       accel_range_plausibility_margin_mps2)
       : m_state_handler{imu_data_storage,
                         imu_health_storage,
                         spi_master,
                         read_failures_limit,
                         execution_period_ms,
                         receive_wait_timeout_ms,
                         sample_rate_divider,
                         dlpf_config,
                         gyro_full_scale,
                         accel_full_scale,
                         accel_a_dlpf_config,
                         gyro_range_plausibility_margin_radps,
                         accel_range_plausibility_margin_mps2}
   {
      spi_master.register_transfer_complete_callback(&Mpu6500::spi_transfer_complete_callback, this);
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

   imu_sensor::ImuSensorError get_error() const
   {
      return m_state_handler.get_error();
   }

private:
   static void spi_transfer_complete_callback(void* ctx)
   {
      auto* self = static_cast<Mpu6500*>(ctx);
      self->notify_receive_complete();
   }

   using StateHandler    = Mpu6500StateHandler<ClockSource, SpiMaster>;
   using StateMachineDef = MainStateMachine<StateHandler>;

   StateHandler                    m_state_handler;
   boost::sml::sm<StateMachineDef> m_state_machine{m_state_handler};
};

}   // namespace mpu6500
