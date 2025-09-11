#pragma once

#include <cmath>

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
                               const std::size_t execution_period_ms,
                               const std::size_t receive_wait_timeout_ms)
       : m_barometer_data_storage{barometer_data_storage},
         m_barometer_health_storage{barometer_health_storage},
         m_i2c_driver{i2c_driver},
         m_logger{logger},
         m_read_failures_limit{read_failures_limit},
         m_execution_period_ms{execution_period_ms},
         m_receive_wait_timeout_ms{receive_wait_timeout_ms}
   {
   }

   void tick_timer()
   {
      m_wait_timer_ms += m_execution_period_ms;
   }

   void reset_timer()
   {
      m_wait_timer_ms = 0;
   }

   void count_read_failure()
   {
      m_local_barometer_health.read_failure_count++;
      m_logger.print("read failure");
   }

   void reset_read_failures()
   {
      m_local_barometer_health.read_failure_count = 0;
   }

   void soft_reset()
   {
      m_tx_buffer.fill(0);
      m_tx_buffer[0] = params::cmd_reg;
      m_tx_buffer[1] = params::CmdReg::soft_reset;
      m_i2c_error    = !m_i2c_driver.write(params::default_i2c_address, std::span{m_tx_buffer.data(), 2u});
   }

   void read_id()
   {
      m_rx_buffer.fill(0);
      m_i2c_error = !m_i2c_driver.read(params::default_i2c_address, std::span{m_rx_buffer.data(), 1u}, params::chip_id_reg);
   }

   void set_oversampling()
   {
      m_tx_buffer.fill(0);
      m_tx_buffer[0] = params::osr_reg;
      m_tx_buffer[1] = params::OsrReg::press_x8_oversampling_mask;   // osr
      m_i2c_error    = !m_i2c_driver.write(params::default_i2c_address, std::span{m_tx_buffer.data(), 2u});
   }

   void set_data_rate()
   {
      m_tx_buffer.fill(0);
      m_tx_buffer[0] = params::odr_reg;
      m_tx_buffer[1] = params::OdrReg::odr_25hz;   // odr
      m_i2c_error    = !m_i2c_driver.write(params::default_i2c_address, std::span{m_tx_buffer.data(), 2u});
   }

   void write_irr_config()
   {
      m_tx_buffer.fill(0);
      m_tx_buffer[0] = params::config_reg;
      m_tx_buffer[1] = params::ConfigReg::iir_coef3_mask;
      m_i2c_error    = !m_i2c_driver.write(params::default_i2c_address, std::span{m_tx_buffer.data(), 2u});
   }

   void set_normal_mode()
   {
      m_tx_buffer.fill(0);
      m_tx_buffer[0] = params::pwr_ctrl_reg;
      m_tx_buffer[1] = params::PwrCtrlReg::pressure_sensor_enable_mask |   // power control
                       params::PwrCtrlReg::temperature_sensor_enable_mask |
                       params::PwrCtrlReg::normal_mode_mask;
      m_i2c_error = !m_i2c_driver.write(params::default_i2c_address, std::span{m_tx_buffer.data(), 2u});
   }

   void read_config_burst()
   {
      m_rx_buffer.fill(0);
      // read 5 registers from 0x1b through 0x1f
      m_i2c_error = !m_i2c_driver.read(params::default_i2c_address, std::span{m_rx_buffer.data(), 5u}, params::pwr_ctrl_reg);
   }

   void read_coefficients()
   {
      m_rx_buffer.fill(0);
      // read 21 bytes of trimmming coefficients
      m_i2c_error = !m_i2c_driver.read(params::default_i2c_address, std::span{m_rx_buffer.data(), params::num_bytes_calibration_data}, params::calibration_data_reg);
   }

   void read_data()
   {
      m_rx_buffer.fill(0);
      m_i2c_error = !m_i2c_driver.read(params::default_i2c_address, std::span{m_rx_buffer.data(), params::num_bytes_data}, params::err_reg);
   }

   void store_id()
   {
      m_device_id = m_rx_buffer[0];
      m_logger.printf("device id: 0x%x", m_device_id);
   }

   void store_config()
   {
      m_config_registers.pwr_ctrl = m_rx_buffer[0];   // 0x1b
      m_config_registers.osr      = m_rx_buffer[1];   // 0x1c
      m_config_registers.odr      = m_rx_buffer[2];   // 0x1d
      m_config_registers.config   = m_rx_buffer[4];   // 0x1f (0x1e -> reserved)
   }

   void store_coefficients()
   {
      // temp coefficients
      m_raw_coefficients.par_t1 = to_uint16(m_rx_buffer[1], m_rx_buffer[0]);
      m_raw_coefficients.par_t2 = to_uint16(m_rx_buffer[3], m_rx_buffer[2]);
      m_raw_coefficients.par_t3 = static_cast<int8_t>(m_rx_buffer[4]);

      m_quantized_coefficients.par_t1 = static_cast<float>(static_cast<float>(m_raw_coefficients.par_t1) * scale_t1);
      m_quantized_coefficients.par_t2 = static_cast<float>(static_cast<float>(m_raw_coefficients.par_t2) * scale_t2);
      m_quantized_coefficients.par_t3 = static_cast<float>(static_cast<float>(m_raw_coefficients.par_t3) * scale_t3);

      // pressure coefficients
      m_raw_coefficients.par_p1  = to_int16(m_rx_buffer[6], m_rx_buffer[5]);
      m_raw_coefficients.par_p2  = to_int16(m_rx_buffer[8], m_rx_buffer[7]);
      m_raw_coefficients.par_p3  = static_cast<int8_t>(m_rx_buffer[9]);
      m_raw_coefficients.par_p4  = static_cast<int8_t>(m_rx_buffer[10]);
      m_raw_coefficients.par_p5  = to_uint16(m_rx_buffer[12], m_rx_buffer[11]);
      m_raw_coefficients.par_p6  = to_uint16(m_rx_buffer[14], m_rx_buffer[13]);
      m_raw_coefficients.par_p7  = static_cast<int8_t>(m_rx_buffer[15]);
      m_raw_coefficients.par_p8  = static_cast<int8_t>(m_rx_buffer[16]);
      m_raw_coefficients.par_p9  = to_int16(m_rx_buffer[18], m_rx_buffer[17]);
      m_raw_coefficients.par_p10 = static_cast<int8_t>(m_rx_buffer[19]);
      m_raw_coefficients.par_p11 = static_cast<int8_t>(m_rx_buffer[20]);

      m_quantized_coefficients.par_p1  = static_cast<float>((static_cast<double>(m_raw_coefficients.par_p1) - offset_p1_p2) * scale_p1);
      m_quantized_coefficients.par_p2  = static_cast<float>((static_cast<double>(m_raw_coefficients.par_p2) - offset_p1_p2) * scale_p2);
      m_quantized_coefficients.par_p3  = static_cast<float>(static_cast<double>(m_raw_coefficients.par_p3) * scale_p3);
      m_quantized_coefficients.par_p4  = static_cast<float>(static_cast<double>(m_raw_coefficients.par_p4) * scale_p4);
      m_quantized_coefficients.par_p5  = static_cast<float>(static_cast<double>(m_raw_coefficients.par_p5) * scale_p5);
      m_quantized_coefficients.par_p6  = static_cast<float>(static_cast<double>(m_raw_coefficients.par_p6) * scale_p6);
      m_quantized_coefficients.par_p7  = static_cast<float>(static_cast<double>(m_raw_coefficients.par_p7) * scale_p7);
      m_quantized_coefficients.par_p8  = static_cast<float>(static_cast<double>(m_raw_coefficients.par_p8) * scale_p8);
      m_quantized_coefficients.par_p9  = static_cast<float>(static_cast<double>(m_raw_coefficients.par_p9) * scale_p9);
      m_quantized_coefficients.par_p10 = static_cast<float>(static_cast<double>(m_raw_coefficients.par_p10) * scale_p10);
      m_quantized_coefficients.par_p11 = static_cast<float>(static_cast<double>(m_raw_coefficients.par_p11) * scale_p11);
   }

   void process_error_register()
   {
      m_error_register = m_rx_buffer[0];
   }

   void convert_raw_data()
   {
      // data is read out in burst from err_register (0x02) -> m_rx_buffer[0]
      uint32_t pressure_raw    = to_int24(m_rx_buffer[4], m_rx_buffer[3], m_rx_buffer[2]);
      uint32_t temperature_raw = to_int24(m_rx_buffer[7], m_rx_buffer[6], m_rx_buffer[5]);

      m_local_barometer_data.temperature_c = compensate_temperature(static_cast<float>(temperature_raw));
      m_local_barometer_data.pressure_pa   = compensate_pressure(m_local_barometer_data.temperature_c.value(), static_cast<float>(pressure_raw));
   }

   void publish_data()
   {
      const volatile uint32_t clock = ClockSource::now_ms();
      m_barometer_data_storage.update_latest(m_local_barometer_data, clock);

      m_data_log_counter++;
      if ((m_data_log_counter % 25u) == 0)
      {
         m_logger.printf("clock: %u   pressure: %.2f     |     temperature: %.2f",
                         clock,
                         m_local_barometer_data.pressure_pa,
                         m_local_barometer_data.temperature_c.value());
      }
   }

   void publish_health()
   {
      m_barometer_health_storage.update_latest(m_local_barometer_health, ClockSource::now_ms());
   }

   void reset_data()
   {
      m_local_barometer_data.pressure_pa = 0.0f;
      m_local_barometer_data.altitude_m  = 0.0f;
      m_local_barometer_data.temperature_c.reset();
      m_barometer_data_storage.update_latest(m_local_barometer_data, ClockSource::now_ms());
   }

   void mark_validation_fail()
   {
      m_local_barometer_health.validation_ok = false;
      m_logger.print("validation failed");
   }

   void mark_validation_success()
   {
      m_local_barometer_health.validation_ok = true;
      m_logger.print("validation successful");
   }

   void mark_config_fail()
   {
      m_local_barometer_health.config_ok = false;
      m_logger.print("config failed");
   }

   void mark_config_success()
   {
      m_local_barometer_health.config_ok = true;
      m_logger.print("config successful");
   }

   void set_state(const barometer_sensor::BarometerSensorState state)
   {
      m_local_barometer_health.state = state;

      switch (m_local_barometer_health.state)
      {
         case barometer_sensor::BarometerSensorState::stopped:
            m_logger.print("entered state->stopped");
            break;
         case barometer_sensor::BarometerSensorState::setup:
            m_logger.print("entered state->setup");
            break;
         case barometer_sensor::BarometerSensorState::self_test:
            m_logger.print("entered state->self_test");
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
            m_logger.print("entered unexpected state");
            error::stop_operation();
            break;
      }
   }

   void set_error(const barometer_sensor::BarometerSensorError error)
   {
      m_local_barometer_health.error.set(static_cast<uint8_t>(error));

      switch (error)
      {
         case barometer_sensor::BarometerSensorError::bus_error:
            m_logger.print("encountered error->bus_error");
            break;
         case barometer_sensor::BarometerSensorError::id_mismatch_error:
            m_logger.print("encountered error->id_mismatch_error");
            break;
         case barometer_sensor::BarometerSensorError::config_mismatch_error:
            m_logger.print("encountered error->config_mismatch_error");
            break;
         case barometer_sensor::BarometerSensorError::sensor_error:
            m_logger.print("encountered error->sensor_error");
            break;
         case barometer_sensor::BarometerSensorError::data_error:
            m_logger.print("encountered error->data_error");
            break;
         case barometer_sensor::BarometerSensorError::max_error:
            break;
         default:
            m_logger.print("encountered unexpected error");
            error::stop_operation();
            break;
      }
   }

   barometer_sensor::BarometerSensorState get_state() const
   {
      return m_local_barometer_health.state;
   }

   barometer_sensor::BarometerHealth::ErrorBits get_error() const
   {
      return m_local_barometer_health.error;
   }

   bool validation_successful() const
   {
      return m_local_barometer_health.validation_ok;
   }

   bool config_successful() const
   {
      return m_local_barometer_health.config_ok;
   }

   bool id_matched() const
   {
      return (m_device_id == params::device_id);
   }

   bool config_matched() const
   {
      const uint8_t pwr_ctrl_config = params::PwrCtrlReg::pressure_sensor_enable_mask | params::PwrCtrlReg::temperature_sensor_enable_mask |
                                      params::PwrCtrlReg::normal_mode_mask;
      const uint8_t osr_config = params::OsrReg::press_x8_oversampling_mask | params::OsrReg::temp_no_oversampling_mask;

      return ((m_config_registers.pwr_ctrl == pwr_ctrl_config) &&
              (m_config_registers.osr == osr_config) &&
              (m_config_registers.odr == params::OdrReg::odr_25hz) &&
              (m_config_registers.config == params::ConfigReg::iir_coef3_mask));
   }

   bool is_buffer_non_zero() const
   {
      for (std::size_t i = 1u; i < params::num_bytes_data; i++)
      {
         if (m_rx_buffer[i] != 0)
         {
            return true;
         }
      }

      return false;
   }

   bool sensor_error_reported() const
   {
      return ((m_error_register & params::ErrReg::fatal_err_mask) ||
              (m_error_register & params::ErrReg::cmd_err_mask) ||
              (m_error_register & params::ErrReg::conf_err_mask));
   }

   static bool is_data_valid()
   {
      return true;
   }

   bool transfer_error() const
   {
      return m_i2c_error;
   }

   bool receive_wait_timeout() const
   {
      return m_wait_timer_ms >= m_receive_wait_timeout_ms;
   }

   bool read_failures_below_limit() const
   {
      return (m_local_barometer_health.read_failure_count < m_read_failures_limit);
   }

private:
   float compensate_temperature(const float uncomp_temp) const
   {
      float partial_data1 = uncomp_temp - m_quantized_coefficients.par_t1;
      float partial_data2 = partial_data1 * m_quantized_coefficients.par_t2;
      return partial_data2 + (partial_data1 * partial_data1) * m_quantized_coefficients.par_t3;
   }

   float compensate_pressure(const float comp_temp, const float uncomp_pressure) const
   {
      float partial_data1 = m_quantized_coefficients.par_p6 * comp_temp;
      float partial_data2 = m_quantized_coefficients.par_p7 * (comp_temp * comp_temp);
      float partial_data3 = m_quantized_coefficients.par_p8 * (comp_temp * comp_temp * comp_temp);
      float partial_out1  = m_quantized_coefficients.par_p5 + partial_data1 + partial_data2 + partial_data3;

      partial_data1      = m_quantized_coefficients.par_p2 * comp_temp;
      partial_data2      = m_quantized_coefficients.par_p3 * (comp_temp * comp_temp);
      partial_data3      = m_quantized_coefficients.par_p4 * (comp_temp * comp_temp * comp_temp);
      float partial_out2 = uncomp_pressure * (m_quantized_coefficients.par_p1 + partial_data1 + partial_data2 + partial_data3);

      partial_data1       = uncomp_pressure * uncomp_pressure;
      partial_data2       = m_quantized_coefficients.par_p9 + m_quantized_coefficients.par_p10 * comp_temp;
      partial_data3       = partial_data1 * partial_data2;
      float partial_data4 = partial_data3 + (uncomp_pressure * uncomp_pressure * uncomp_pressure) * m_quantized_coefficients.par_p11;

      return (partial_out1 + partial_out2 + partial_data4);
   }

   static constexpr uint32_t to_int24(const uint32_t msb, const uint32_t lsb, const uint32_t xlsb) noexcept
   {
      return (static_cast<uint32_t>(msb << two_bytes_shift) |
              static_cast<uint32_t>(lsb << one_byte_shift) |
              xlsb);
   }

   static constexpr uint16_t to_uint16(const uint8_t msb, const uint8_t lsb) noexcept
   {
      return (static_cast<uint16_t>(msb << one_byte_shift) | lsb);
   }

   static constexpr int16_t to_int16(const uint8_t msb, const uint8_t lsb) noexcept
   {
      uint16_t u = to_uint16(msb, lsb);
      return (static_cast<int16_t>(u));
   }

   struct Config_registers
   {
      uint8_t pwr_ctrl;
      uint8_t osr;
      uint8_t odr;
      uint8_t config;
   };

   static constexpr uint8_t two_bytes_shift = 16u;
   static constexpr uint8_t one_byte_shift  = 8u;

   static constexpr double scale_t1 = std::ldexp(1.0, 8);        // dividy by 2^-8, for multiplication invert the sign
   static constexpr double scale_t2 = std::ldexp(1.0, -30);      // dividy by 2^30, for multiplication invert the sign
   static constexpr double scale_t3 = std::ldexp(1.0, -48);      // dividy by 2^48, for multiplication invert the sign

   static constexpr double scale_p1  = std::ldexp(1.0, -20);     // dividy by 2^20, for multiplication invert the sign
   static constexpr double scale_p2  = std::ldexp(1.0, -29);     // dividy by 2^29, for multiplication invert the sign
   static constexpr double scale_p3  = std::ldexp(1.0, -32);     // dividy by 2^32, for multiplication invert the sign
   static constexpr double scale_p4  = std::ldexp(1.0, -37);     // dividy by 2^37, for multiplication invert the sign
   static constexpr double scale_p5  = std::ldexp(1.0, 3);       // dividy by 2^-3, for multiplication invert the sign
   static constexpr double scale_p6  = std::ldexp(1.0, -6);      // dividy by 2^6, for multiplication invert the sign
   static constexpr double scale_p7  = std::ldexp(1.0, -8);      // dividy by 2^8, for multiplication invert the sign
   static constexpr double scale_p8  = std::ldexp(1.0, -15);     // dividy by 2^15, for multiplication invert the sign
   static constexpr double scale_p9  = std::ldexp(1.0, -48);     // dividy by 2^48, for multiplication invert the sign
   static constexpr double scale_p10 = std::ldexp(1.0, -48);     // dividy by 2^48, for multiplication invert the sign
   static constexpr double scale_p11 = std::ldexp(1.0, -65);     // dividy by 2^65, for multiplication invert the sign

   static constexpr double offset_p1_p2 = std::ldexp(1.0, 14);   // 2^14 - no sign inversion

   BarometerData&                           m_barometer_data_storage;
   BarometerHealth&                         m_barometer_health_storage;
   I2cDriver&                               m_i2c_driver;
   Logger&                                  m_logger;
   const uint8_t                            m_read_failures_limit;
   const std::size_t                        m_execution_period_ms;
   const std::size_t                        m_receive_wait_timeout_ms;
   barometer_sensor::BarometerData          m_local_barometer_data{};
   barometer_sensor::BarometerHealth        m_local_barometer_health{};
   std::array<uint8_t, params::buffer_size> m_tx_buffer{};
   std::array<uint8_t, params::buffer_size> m_rx_buffer{};
   uint8_t                                  m_device_id{};
   Config_registers                         m_config_registers{};
   params::CalibrationCoeffiecients         m_raw_coefficients{};
   params::QuantizedCoeffiecients           m_quantized_coefficients{};
   uint8_t                                  m_error_register{};
   bool                                     m_i2c_error{false};
   std::size_t                              m_wait_timer_ms{};
   std::size_t                              m_data_log_counter{};
};

}   // namespace bmp390
