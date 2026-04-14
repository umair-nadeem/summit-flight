#pragma once

#include <algorithm>
#include <cmath>
#include <span>

#include "SensorState.hpp"
#include "error/error_handler.hpp"
#include "imu_sensor/ImuSensorStatus.hpp"
#include "imu_sensor/RawImuSensorData.hpp"
#include "math/Vector3.hpp"
#include "math/constants.hpp"
#include "params.hpp"

namespace mpu6500
{

template <typename SpiMaster, typename Logger>
class Mpu6500StateHandler
{
   using Vec3 = math::Vec3f;

public:
   explicit Mpu6500StateHandler(SpiMaster&                    spi_master,
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
       : m_spi_master{spi_master},
         m_logger{logger},
         m_raw_sensor_data_out{raw_sensor_data_out},
         m_sensor_status_out{sensor_status_out},
         m_read_failures_limit{read_failures_limit},
         m_execution_period_ms{execution_period_ms},
         m_receive_wait_timeout_ms{receive_wait_timeout_ms},
         m_sample_rate_divider{sample_rate_divider},
         m_dlpf_config{dlpf_config},
         m_gyro_full_scale{gyro_full_scale},
         m_accel_full_scale{accel_full_scale},
         m_accel_a_dlpf_config{accel_a_dlpf_config},
         m_gyro_range_plausibility_margin_radps{gyro_range_plausibility_margin_radps},
         m_accel_range_plausibility_margin_mps2{accel_range_plausibility_margin_mps2},
         m_gyro_scale{math::constants::deg_to_rad / params::gyro_sensitivity_scale_factor[m_gyro_full_scale]},
         m_accel_scale{math::constants::g_to_mps2 / params::accel_sensitivity_scale_factor[m_accel_full_scale]},
         m_gyro_absolute_plausibility_limit_radps{(params::gyro_abs_full_scale_range_dps[m_gyro_full_scale] * math::constants::deg_to_rad) + m_gyro_range_plausibility_margin_radps},
         m_accel_absolute_plausibility_limit_mps2{(params::accel_abs_full_scale_range_g[m_accel_full_scale] * math::constants::g_to_mps2) + m_accel_range_plausibility_margin_mps2}
   {
      m_logger.enable();
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
      m_sensor_status.read_failure_count++;
      m_logger.print("read failure");
   }

   void reset_read_failures()
   {
      m_sensor_status.read_failure_count = 0;
   }

   void power_reset()
   {
      m_tx_buffer[0] = get_write_spi_command(params::pwr_mgmt_1_reg);
      m_tx_buffer[1] = params::PwrMgmt1BitMask::device_reset;
      m_spi_master.transfer(std::span{m_tx_buffer.data(), 2u}, std::span{m_rx_buffer.data(), 2u});
   }

   void signal_path_reset()
   {
      m_tx_buffer[0] = get_write_spi_command(params::signal_path_reset_reg);
      m_tx_buffer[1] = params::SignalPathResetBitMask::gyro_reset | params::SignalPathResetBitMask::accel_reset | params::SignalPathResetBitMask::temp_reset;
      m_spi_master.transfer(std::span{m_tx_buffer.data(), 2u}, std::span{m_rx_buffer.data(), 2u});
   }

   void disable_i2c()
   {
      m_tx_buffer[0] = get_write_spi_command(params::user_ctrl_reg);
      m_tx_buffer[1] = params::UserCtrlBitMask::i2c_if_dis;
      m_spi_master.transfer(std::span{m_tx_buffer.data(), 2u}, std::span{m_rx_buffer.data(), 2u});
   }

   void set_clock_and_wakeup()
   {
      m_tx_buffer[0] = get_write_spi_command(params::pwr_mgmt_1_reg);
      m_tx_buffer[1] = params::PwrMgmt1BitMask::clock_sel;
      m_spi_master.transfer(std::span{m_tx_buffer.data(), 2u}, std::span{m_rx_buffer.data(), 2u});
   }

   void read_id()
   {
      m_tx_buffer.fill(0);
      m_rx_buffer.fill(0);
      m_tx_buffer[0] = get_read_spi_command(params::who_am_i_reg);
      m_tx_buffer[1] = 0;
      m_spi_master.transfer(std::span{m_tx_buffer.data(), 2u}, std::span{m_rx_buffer.data(), 2u});
   }

   void write_config_burst()
   {
      constexpr std::size_t burst_len = 1u      // address
                                        + 5u;   // registers

      // write to sample rate, config, gyro config and accel config registers in single burst
      m_tx_buffer[0] = get_write_spi_command(params::smplrt_div_reg);
      m_tx_buffer[1] = m_sample_rate_divider;                                                                  // sample rate
      m_tx_buffer[2] = m_dlpf_config & params::ConfigBitMask::dlpf_cfg;                                        // config reg
      m_tx_buffer[3] = ((m_gyro_full_scale << three_bit_shift) & params::GyroConfigBitMask::gyro_fs_sel);      // gyro config reg, FCHOICE_B[1:0]=0
      m_tx_buffer[4] = ((m_accel_full_scale << three_bit_shift) & params::AccelConfigBitMask::accel_fs_sel);   // accel config reg
      m_tx_buffer[5] = m_accel_a_dlpf_config & params::AccelConfig2BitMask::accel_a_dlpf_config;               // accel config 2 reg, ACCEL_FCHOICE_B=0

      m_spi_master.transfer(std::span{m_tx_buffer.data(), burst_len}, std::span{m_rx_buffer.data(), burst_len});
   }

   void read_config_burst()
   {
      m_tx_buffer.fill(0);
      m_rx_buffer.fill(0);
      constexpr std::size_t burst_len = 1u      // address
                                        + 5u;   // registers

      m_tx_buffer[0] = get_read_spi_command(params::smplrt_div_reg);
      m_tx_buffer[1] = 0x00;
      m_tx_buffer[2] = 0x00;
      m_tx_buffer[3] = 0x00;
      m_tx_buffer[4] = 0x00;
      m_tx_buffer[5] = 0x00;
      m_spi_master.transfer(std::span{m_tx_buffer.data(), burst_len}, std::span{m_rx_buffer.data(), burst_len});
   }

   void read_data()
   {
      m_tx_buffer.fill(0);
      m_rx_buffer.fill(0);
      m_tx_buffer[0] = get_read_spi_command(params::accel_xout_h_reg);
      m_spi_master.transfer(std::span{m_tx_buffer}, std::span{m_rx_buffer});
   }

   void store_id()
   {
      m_device_id = m_rx_buffer[1];
      m_logger.printf("device id: 0x%x", m_device_id);
   }

   void store_config()
   {
      m_config_registers.sample_rate_div = m_rx_buffer[1];
      m_config_registers.config          = m_rx_buffer[2];
      m_config_registers.gyro_config     = m_rx_buffer[3];
      m_config_registers.accel_config    = m_rx_buffer[4];
      m_config_registers.accel_config_2  = m_rx_buffer[5];
   }

   void convert_raw_data()
   {
      m_raw_sensor_data.accel_mps2 = Vec3{to_int16(m_rx_buffer[1], m_rx_buffer[2]) * m_accel_scale,
                                          to_int16(m_rx_buffer[3], m_rx_buffer[4]) * m_accel_scale,
                                          to_int16(m_rx_buffer[5], m_rx_buffer[6]) * m_accel_scale};

      m_raw_sensor_data.temperature_c = (to_int16(m_rx_buffer[7], m_rx_buffer[8]) / params::temp_sensitivity) + params::temp_offset;

      m_raw_sensor_data.gyro_radps = Vec3{to_int16(m_rx_buffer[9], m_rx_buffer[10]) * m_gyro_scale,
                                          to_int16(m_rx_buffer[11], m_rx_buffer[12]) * m_gyro_scale,
                                          to_int16(m_rx_buffer[13], m_rx_buffer[14]) * m_gyro_scale};
   }

   void update()
   {
      // atomically update output
      m_raw_sensor_data.count++;
      m_raw_sensor_data_out = m_raw_sensor_data;
      m_sensor_status_out   = m_sensor_status;
   }

   void reset_data()
   {
      m_raw_sensor_data.accel_mps2.zero();
      m_raw_sensor_data.gyro_radps.zero();
      m_raw_sensor_data.temperature_c.reset();
   }

   void mark_validation_fail()
   {
      m_sensor_status.validation_ok = false;
      m_logger.print("validation failed");
   }

   void mark_validation_success()
   {
      m_sensor_status.validation_ok = true;
      m_logger.print("validation successful");
   }

   void mark_config_fail()
   {
      m_sensor_status.config_ok = false;
      m_logger.print("config failed");
   }

   void mark_config_success()
   {
      m_sensor_status.config_ok = true;
      m_logger.print("config successful");
   }

   void set_fault()
   {
      m_sensor_status.fault = true;
      set_state(mpu6500::SensorState::fault);
   }

   void set_state(const mpu6500::SensorState& state)
   {
      m_state = state;

      switch (m_state)
      {
         case mpu6500::SensorState::stopped:
            m_logger.print("entered state->stopped");
            break;
         case mpu6500::SensorState::reset:
            m_logger.print("entered state->reset");
            break;
         case mpu6500::SensorState::validation:
            m_logger.print("entered state->validation");
            break;
         case mpu6500::SensorState::config:
            m_logger.print("entered state->config");
            break;
         case mpu6500::SensorState::operational:
            m_logger.print("entered state->operational");
            break;
         case mpu6500::SensorState::soft_recovery:
            m_logger.print("entered state->soft_recovery");
            break;
         case mpu6500::SensorState::hard_recovery:
            m_logger.print("entered state->hard_recovery");
            break;
         case mpu6500::SensorState::fault:
            m_logger.print("entered state->fault");
            break;
         default:
            m_logger.print("entered unexpected state");
            error::stop_operation();
            break;
      }
   }

   void set_error(const imu_sensor::ImuSensorError& error)
   {
      m_sensor_status.error.set(static_cast<types::ErrorBitsType>(error));

      switch (error)
      {
         case imu_sensor::ImuSensorError::bus_error:
            m_logger.print("encountered error->bus_error");
            break;
         case imu_sensor::ImuSensorError::id_mismatch_error:
            m_logger.print("encountered error->id_mismatch_error");
            break;
         case imu_sensor::ImuSensorError::config_mismatch_error:
            m_logger.print("encountered error->config_mismatch_error");
            break;
         case imu_sensor::ImuSensorError::data_pattern_error:
            m_logger.print("encountered error->data_pattern_error");
            break;
         case imu_sensor::ImuSensorError::out_of_range_data_error:
            m_logger.print("encountered error->out_of_range_data_error");
            break;
         case imu_sensor::ImuSensorError::max_error:
         default:
            m_logger.print("encountered unexpected error");
            error::stop_operation();
            break;
      }
   }

   imu_sensor::RawImuSensorData get_raw_data() const
   {
      return m_raw_sensor_data;
   }

   imu_sensor::ImuSensorStatus get_status() const
   {
      return m_sensor_status;
   }

   imu_sensor::ImuSensorErrorBits get_error() const
   {
      return m_sensor_status.error;
   }

   mpu6500::SensorState get_state() const
   {
      return m_state;
   }

   bool validation_successful() const
   {
      return m_sensor_status.validation_ok;
   }

   bool config_successful() const
   {
      return m_sensor_status.config_ok;
   }

   bool id_matched() const
   {
      return (m_device_id == params::device_id);
   }

   bool config_matched() const
   {
      const uint8_t dlpf_config         = m_config_registers.config & params::ConfigBitMask::dlpf_cfg;
      const uint8_t gyro_fs_sel         = (m_config_registers.gyro_config & params::GyroConfigBitMask::gyro_fs_sel) >> three_bit_shift;
      const uint8_t accel_fs_sel        = (m_config_registers.accel_config & params::AccelConfigBitMask::accel_fs_sel) >> three_bit_shift;
      const uint8_t accel_a_dlpf_config = m_config_registers.accel_config_2 & params::AccelConfig2BitMask::accel_a_dlpf_config;

      return ((m_config_registers.sample_rate_div == m_sample_rate_divider) &&
              (dlpf_config == m_dlpf_config) &&
              (gyro_fs_sel == m_gyro_full_scale) &&
              (accel_fs_sel == m_accel_full_scale) &&
              (accel_a_dlpf_config == m_accel_a_dlpf_config));
   }

   bool is_data_pattern_ok() const
   {
      const auto buffer = std::span{m_rx_buffer.data(), params::num_bytes_transaction};
      return (!is_buffer_all_zeros(buffer) && !is_buffer_all_ones(buffer));
   }

   bool is_data_valid() const
   {
      return (is_gyro_range_plausible() && is_accel_range_plausible() && is_temp_plausible());
   }

   bool power_reset_wait_over() const
   {
      return m_wait_timer_ms >= params::power_on_reset_wait_ms;
   }

   bool signal_reset_wait_over() const
   {
      return m_wait_timer_ms >= params::signal_path_reset_wait_ms;
   }

   bool receive_wait_timeout() const
   {
      return m_wait_timer_ms >= m_receive_wait_timeout_ms;
   }

   bool read_failures_below_limit() const
   {
      return (m_sensor_status.read_failure_count < m_read_failures_limit);
   }

private:
   static constexpr uint8_t get_write_spi_command(const uint8_t reg) noexcept
   {
      return (reg & static_cast<uint8_t>(~params::read_mask));
   }

   static constexpr uint8_t get_read_spi_command(const uint8_t reg) noexcept
   {
      return (reg | params::read_mask);
   }

   static constexpr bool is_buffer_all_zeros(std::span<const uint8_t> buffer) noexcept
   {
      return std::all_of(buffer.begin(), buffer.end(), [](const uint8_t v)
                         { return v == 0; });
   }

   static constexpr bool is_buffer_all_ones(std::span<const uint8_t> buffer) noexcept
   {
      return std::all_of(buffer.begin(), buffer.end(), [](const uint8_t v)
                         { return v == 0xff; });
   }

   static constexpr auto to_int16(const uint8_t msb, const uint8_t lsb) noexcept
   {
      return static_cast<int16_t>((msb << one_byte_shift) | lsb);
   }

   bool is_gyro_range_plausible() const
   {
      // sensor range check -> must be performed with raw/uncalibrated data
      const auto& g = m_raw_sensor_data.gyro_radps;
      return ((std::fabs(g[0]) <= m_gyro_absolute_plausibility_limit_radps) &&
              (std::fabs(g[1]) <= m_gyro_absolute_plausibility_limit_radps) &&
              (std::fabs(g[2]) <= m_gyro_absolute_plausibility_limit_radps));
   }

   bool is_accel_range_plausible() const
   {
      // sensor range check -> must be performed with raw/uncalibrated data
      const auto& a = m_raw_sensor_data.accel_mps2;
      return ((std::fabs(a[0]) <= m_accel_absolute_plausibility_limit_mps2) &&
              (std::fabs(a[1]) <= m_accel_absolute_plausibility_limit_mps2) &&
              (std::fabs(a[2]) <= m_accel_absolute_plausibility_limit_mps2));
   }

   bool is_temp_plausible() const
   {
      // sensor range check -> must be performed with raw/uncalibrated data
      if (!m_raw_sensor_data.temperature_c.has_value())
      {
         return true;   // nothing to check
      }

      return ((params::temp_min_plauisble_range < m_raw_sensor_data.temperature_c.value()) &&
              (m_raw_sensor_data.temperature_c.value() < params::temp_max_plauisble_range));
   }

   struct Config_registers
   {
      uint8_t sample_rate_div;
      uint8_t config;
      uint8_t gyro_config;
      uint8_t accel_config;
      uint8_t accel_config_2;
   };

   static constexpr uint8_t one_byte_shift  = 8u;
   static constexpr uint8_t three_bit_shift = 3u;

   SpiMaster&                                         m_spi_master;
   Logger&                                            m_logger;
   imu_sensor::RawImuSensorData&                      m_raw_sensor_data_out;
   imu_sensor::ImuSensorStatus&                       m_sensor_status_out;
   const uint8_t                                      m_read_failures_limit;
   const uint32_t                                     m_execution_period_ms;
   const uint32_t                                     m_receive_wait_timeout_ms;
   const uint8_t                                      m_sample_rate_divider;
   const uint8_t                                      m_dlpf_config;
   const uint8_t                                      m_gyro_full_scale;
   const uint8_t                                      m_accel_full_scale;
   const uint8_t                                      m_accel_a_dlpf_config;
   const float                                        m_gyro_range_plausibility_margin_radps;
   const float                                        m_accel_range_plausibility_margin_mps2;
   const float                                        m_gyro_scale;
   const float                                        m_accel_scale;
   const float                                        m_gyro_absolute_plausibility_limit_radps;
   const float                                        m_accel_absolute_plausibility_limit_mps2;
   std::array<uint8_t, params::num_bytes_transaction> m_tx_buffer{};
   std::array<uint8_t, params::num_bytes_transaction> m_rx_buffer{};
   imu_sensor::RawImuSensorData                       m_raw_sensor_data{};
   imu_sensor::ImuSensorStatus                        m_sensor_status{};
   mpu6500::SensorState                               m_state{mpu6500::SensorState::stopped};
   uint8_t                                            m_device_id{};
   Config_registers                                   m_config_registers{};
   uint32_t                                           m_wait_timer_ms{};
   std::size_t                                        m_data_log_counter{};
};

}   // namespace mpu6500
