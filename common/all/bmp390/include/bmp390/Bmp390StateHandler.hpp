#pragma once

#include "barometer_sensor/BarometerData.hpp"
#include "barometer_sensor/BarometerHealth.hpp"
#include "boundaries/SharedData.hpp"
#include "interfaces/IClockSource.hpp"
#include "params.hpp"

namespace bmp390
{

template <interfaces::IClockSource ClockSource, typename SpiMaster, typename Logger>
class Bmp390StateHandler
{
   using BarometerData   = ::boundaries::SharedData<barometer_sensor::BarometerData>;
   using BarometerHealth = ::boundaries::SharedData<barometer_sensor::BarometerHealth>;

public:
   explicit Bmp390StateHandler(BarometerData&    barometer_data_storage,
                               BarometerHealth&  barometer_health_storage,
                               SpiMaster&        spi_master,
                               Logger&           logger,
                               const uint8_t     read_failures_limit,
                               const std::size_t execution_period_ms)
       : m_barometer_data_storage{barometer_data_storage},
         m_barometer_health_storage{barometer_health_storage},
         m_spi_master{spi_master},
         m_logger{logger},
         m_read_failures_limit{read_failures_limit},
         m_execution_period_ms{execution_period_ms}
   {
   }

   void reset()
   {
      m_spi_error = false;
   }

   void tick_timer()
   {
      m_state = true;
   }

   void read_id()
   {
      m_tx_buffer[0] = 0x80 | 0x00;
      m_tx_buffer[1] = 0x00;
      m_spi_error    = !m_spi_master.transfer8(std::span{m_tx_buffer.data(), 2u}, std::span{m_rx_buffer.data(), 2u}, 2u, 0);
      // m_spi_master.spi2_test();
   }

   void store_id()
   {
      m_device_id = m_rx_buffer[1];
      m_logger.printf("device id: 0x%x", m_device_id);
   }

   void set_config_state()
   {
      m_state = true;
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
      return (m_rx_buffer[1] == params::device_id);
   }

   bool bus_error() const
   {
      return m_spi_error;
   }

private:
   BarometerData&                                     m_barometer_data_storage;
   BarometerHealth&                                   m_barometer_health_storage;
   SpiMaster&                                         m_spi_master;
   Logger&                                            m_logger;
   const uint8_t                                      m_read_failures_limit;
   const std::size_t                                  m_execution_period_ms;
   barometer_sensor::BarometerData                    m_local_barometer_data{};
   barometer_sensor::BarometerHealth                  m_local_barometer_health{};
   std::array<uint8_t, params::num_bytes_transaction> m_tx_buffer{};
   std::array<uint8_t, params::num_bytes_transaction> m_rx_buffer{};
   uint8_t                                            m_device_id{};
   bool                                               m_spi_error{false};
   bool                                               m_state{false};
};

}   // namespace bmp390
