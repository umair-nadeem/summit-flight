#pragma once

#include "Events.hpp"
#include "MainSM.hpp"
#include "Mpu6500StateHandler.hpp"

namespace mpu6500
{

template <typename SpiMaster, typename Logger>
class Mpu6500
{
public:
   explicit Mpu6500(SpiMaster&                    spi_master,
                    Logger&                       logger,
                    imu_sensor::RawImuSensorData& raw_sensor_data_out,
                    imu_sensor::ImuSensorStatus&  sensor_status_out,
                    const uint8_t                 read_failures_limit,
                    const uint32_t                execution_period_ms,
                    const uint32_t                receive_wait_timeout_ms,
                    const uint8_t                 sample_rate_divider,
                    const uint8_t                 dlpf_config,
                    const uint8_t                 gyro_full_scale,
                    const uint8_t                 accel_full_scale,
                    const uint8_t                 accel_a_dlpf_config,
                    const float                   gyro_range_plausibility_margin_radps,
                    const float                   accel_range_plausibility_margin_mps2)
       : m_state_handler{spi_master,
                         logger,
                         raw_sensor_data_out,
                         sensor_status_out,
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

   imu_sensor::RawImuSensorData get_raw_data() const
   {
      return m_state_handler.get_raw_data();
   }

   imu_sensor::ImuSensorStatus get_status() const
   {
      return m_state_handler.get_status();
   }

   imu_sensor::ImuSensorErrorBits get_error() const
   {
      return m_state_handler.get_error();
   }

   mpu6500::SensorState get_state() const
   {
      return m_state_handler.get_state();
   }

private:
   using StateHandler    = Mpu6500StateHandler<SpiMaster, Logger>;
   using StateMachineDef = MainStateMachine<StateHandler>;

   StateHandler                    m_state_handler;
   boost::sml::sm<StateMachineDef> m_state_machine{m_state_handler};
};

}   // namespace mpu6500
