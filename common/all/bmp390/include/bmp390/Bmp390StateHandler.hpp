#pragma once

#include "barometer_sensor/BarometerData.hpp"
#include "barometer_sensor/BarometerHealth.hpp"
#include "boundaries/SharedData.hpp"
#include "error/error_handler.hpp"
#include "interfaces/IClockSource.hpp"
#include "params.hpp"

namespace bmp390
{

template <interfaces::IClockSource ClockSource, typename I2cDriver, typename Logger>
class Bmp390StateHandler
{
   using BarometerData   = ::boundaries::SharedData<barometer_sensor::BarometerData>;
   using BarometerHealth = ::boundaries::SharedData<barometer_sensor::BarometerHealth>;

public:
   explicit Bmp390StateHandler(BarometerData&    barometer_data_storage,
                               BarometerHealth&  barometer_health_storage,
                               I2cDriver&        i2c_driver,
                               Logger&           logger,
                               const uint8_t     read_failures_limit,
                               const std::size_t execution_period_ms)
       : m_barometer_data_storage{barometer_data_storage},
         m_barometer_health_storage{barometer_health_storage},
         m_i2c_driver{i2c_driver},
         m_logger{logger},
         m_read_failures_limit{read_failures_limit},
         m_execution_period_ms{execution_period_ms}
   {
   }

   void reset()
   {
      m_i2c_error = false;
   }

   void tick_timer()
   {
      m_state = true;
   }

   void read_id()
   {
      m_i2c_error = !m_i2c_driver.read(params::default_i2c_address, std::span{m_rx_buffer.data(), 1u}, 0x00);
   }

   void read_data()
   {
      m_i2c_error = !m_i2c_driver.read(params::default_i2c_address, std::span{m_rx_buffer.data(), 6u}, 0x04);
   }

   void store_id()
   {
      m_device_id = m_rx_buffer[1];
      m_logger.printf("device id: 0x%x", m_device_id);
   }

   void convert_raw_data()
   {
      uint32_t press_raw = (static_cast<uint32_t>(m_rx_buffer[2]) << 16) |
                           (static_cast<uint32_t>(m_rx_buffer[1]) << 8) |
                           (static_cast<uint32_t>(m_rx_buffer[0]));

      uint32_t temp_raw = (static_cast<uint32_t>(m_rx_buffer[5]) << 16) |
                          (static_cast<uint32_t>(m_rx_buffer[4]) << 8) |
                          (static_cast<uint32_t>(m_rx_buffer[3]));

      m_local_barometer_data.pressure_hpa  = static_cast<float>(press_raw >> 4u);
      m_local_barometer_data.temperature_c = static_cast<float>(temp_raw >> 4u);

      const volatile uint32_t clock = ClockSource::now_ms();
      m_data_log_counter++;
      if ((m_data_log_counter % 25u) == 0)
      {
         m_logger.printf("clock: %u   pressure: %.2f     |     temperature: %.2f",
                         clock,
                         m_local_barometer_data.pressure_hpa,
                         m_local_barometer_data.temperature_c.value());
      }
   }

   void set_state(const barometer_sensor::BarometerSensorState state)
   {
      m_local_barometer_health.state = state;

      switch (m_local_barometer_health.state)
      {
         case barometer_sensor::BarometerSensorState::stopped:
            m_logger.print("entered state->stopped");
            break;
         case barometer_sensor::BarometerSensorState::reset:
            m_logger.print("entered state->reset");
            break;
         case barometer_sensor::BarometerSensorState::validation:
            m_logger.print("entered state->validation");
            break;
         case barometer_sensor::BarometerSensorState::self_test:
            m_logger.print("entered state->self_test");
            break;
         case barometer_sensor::BarometerSensorState::config:
            m_logger.print("entered state->config");
            break;
         case barometer_sensor::BarometerSensorState::operational:
            m_logger.print("entered state->operational");
            break;
         case barometer_sensor::BarometerSensorState::soft_recovery:
            m_logger.print("entered state->soft_recovery");
            break;
         case barometer_sensor::BarometerSensorState::hard_recovery:
            m_logger.print("entered state->hard_recovery");
            break;
         case barometer_sensor::BarometerSensorState::failure:
            m_logger.print("entered state->failure");
            break;
         default:
            error::stop_operation();
            break;
      }
   }

   void set_error(const barometer_sensor::BarometerSensorError error)
   {
      m_local_barometer_health.error = error;

      switch (m_local_barometer_health.error)
      {
         case barometer_sensor::BarometerSensorError::bus_error:
            m_logger.print("encountered error->bus_error");
            break;
         case barometer_sensor::BarometerSensorError::sensor_error:
            m_logger.print("entered state->sensor_error");
            break;
         case barometer_sensor::BarometerSensorError::data_error:
            m_logger.print("entered state->data_error");
            break;
         case barometer_sensor::BarometerSensorError::none:
            m_logger.print("entered state->none");
            break;
         default:
            error::stop_operation();
            break;
      }
   }

   bool id_matched() const
   {
      return (m_rx_buffer[0] == params::device_id);
   }

   bool bus_error() const
   {
      return m_i2c_error;
   }

private:
   BarometerData&                                     m_barometer_data_storage;
   BarometerHealth&                                   m_barometer_health_storage;
   I2cDriver&                                         m_i2c_driver;
   Logger&                                            m_logger;
   const uint8_t                                      m_read_failures_limit;
   const std::size_t                                  m_execution_period_ms;
   barometer_sensor::BarometerData                    m_local_barometer_data{};
   barometer_sensor::BarometerHealth                  m_local_barometer_health{};
   std::array<uint8_t, params::num_bytes_transaction> m_tx_buffer{};
   std::array<uint8_t, params::num_bytes_transaction> m_rx_buffer{};
   uint8_t                                            m_device_id{};
   bool                                               m_i2c_error{false};
   bool                                               m_state{false};
   std::size_t                                        m_data_log_counter{};
};

}   // namespace bmp390
