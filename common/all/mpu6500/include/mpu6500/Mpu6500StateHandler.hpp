#pragma once

#include <algorithm>
#include <cmath>
#include <span>

#include "boundaries/SharedData.hpp"
#include "error/error_handler.hpp"
#include "imu_sensor/ImuData.hpp"
#include "imu_sensor/ImuHealth.hpp"
#include "interfaces/IClockSource.hpp"
#include "params.hpp"

namespace mpu6500
{

template <interfaces::IClockSource ClockSource, typename SpiMaster, typename Logger>
class Mpu6500StateHandler
{
   using ImuData   = ::boundaries::SharedData<imu_sensor::ImuData>;
   using ImuHealth = ::boundaries::SharedData<imu_sensor::ImuHealth>;
   using Vec       = imu_sensor::ImuData::Vec3;

public:
   explicit Mpu6500StateHandler(ImuData&          imu_data_storage,
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
                                const uint8_t     num_samples_self_test,
                                const float       gyro_tolerance_radps,
                                const float       accel_tolerance_mps2)
       : m_imu_data_storage{imu_data_storage},
         m_imu_health_storage{imu_health_storage},
         m_spi_master{spi_master},
         m_logger{logger},
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
         m_gyro_scale{deg_to_rad / params::gyro_sensitivity_scale_factor[m_gyro_full_scale]},
         m_accel_scale{g_to_mps2 / params::accel_sensitivity_scale_factor[m_accel_full_scale]},
         m_gyro_absolute_plausibility_limit_radps{(params::gyro_abs_full_scale_range_dps[m_gyro_full_scale] * deg_to_rad) + m_gyro_range_plausibility_margin_radps},
         m_accel_absolute_plausibility_limit_mps2{(params::accel_abs_full_scale_range_g[m_accel_full_scale] * g_to_mps2) + m_accel_range_plausibility_margin_mps2},
         m_num_samples_self_test{num_samples_self_test},
         m_gyro_tolerance_radps{gyro_tolerance_radps},
         m_accel_tolerance_mps2{accel_tolerance_mps2}
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
      m_local_imu_health.read_failure_count++;
      m_logger.print("read failure");
   }

   void reset_read_failures()
   {
      m_local_imu_health.read_failure_count = 0;
   }

   void reset_stats()
   {
      m_self_test_stats.mean_accel = Vec{0, 0, 0};
      m_self_test_stats.mean_gyro  = Vec{0, 0, 0};

      m_self_test_stats.ssq_accel = Vec{0, 0, 0};
      m_self_test_stats.ssq_gyro  = Vec{0, 0, 0};

      m_self_test_stats.std_accel = Vec{0, 0, 0};
      m_self_test_stats.std_gyro  = Vec{0, 0, 0};
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
      m_tx_buffer[3] = ((m_gyro_full_scale << three_bit_shift) & params::GyroConfigBitMask::gyro_fs_sel);      // gyro config reg
      m_tx_buffer[4] = ((m_accel_full_scale << three_bit_shift) & params::AccelConfigBitMask::accel_fs_sel);   // accel config reg
      m_tx_buffer[5] = m_accel_a_dlpf_config & params::AccelConfig2BitMask::accel_a_dlpf_config;               // accel config 2 reg

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

   void update_stats()
   {
      // update step of welford's algorithm
      m_self_test_stats.sample_counter++;   // increment counter

      const Vec& new_accel_data = m_local_imu_data.accel_mps2.value();
      const Vec& new_gyro_data  = m_local_imu_data.gyro_radps.value();

      const Vec delta_accel = new_accel_data - m_self_test_stats.mean_accel;
      const Vec delta_gyro  = new_gyro_data - m_self_test_stats.mean_gyro;

      const float n              = static_cast<float>(m_self_test_stats.sample_counter);
      const Vec   new_mean_accel = m_self_test_stats.mean_accel + (delta_accel / n);
      const Vec   new_mean_gyro  = m_self_test_stats.mean_gyro + (delta_gyro / n);

      m_self_test_stats.ssq_accel = m_self_test_stats.ssq_accel + (delta_accel * (new_accel_data - new_mean_accel));
      m_self_test_stats.ssq_gyro  = m_self_test_stats.ssq_gyro + (delta_gyro * (new_gyro_data - new_mean_gyro));

      m_self_test_stats.mean_accel = new_mean_accel;
      m_self_test_stats.mean_gyro  = new_mean_gyro;
   }

   void calculate_stats_and_bias()
   {
      calculate_standard_deviation();

      calculate_bias();
   }

   void convert_raw_data()
   {
      m_local_imu_data.accel_mps2 = Vec{to_int16(m_rx_buffer[1], m_rx_buffer[2]) * m_accel_scale,
                                        to_int16(m_rx_buffer[3], m_rx_buffer[4]) * m_accel_scale,
                                        to_int16(m_rx_buffer[5], m_rx_buffer[6]) * m_accel_scale};

      m_local_imu_data.temperature_c = (to_int16(m_rx_buffer[7], m_rx_buffer[8]) / params::temp_sensitivity) + params::temp_offset;

      m_local_imu_data.gyro_radps = Vec{to_int16(m_rx_buffer[9], m_rx_buffer[10]) * m_gyro_scale,
                                        to_int16(m_rx_buffer[11], m_rx_buffer[12]) * m_gyro_scale,
                                        to_int16(m_rx_buffer[13], m_rx_buffer[14]) * m_gyro_scale};
   }

   void adjust_bias()
   {
      // subtract bias from gyro and accel data
      m_local_imu_data.accel_mps2 = Vec{m_local_imu_data.accel_mps2.value() - m_bias.accel};
      m_local_imu_data.gyro_radps = Vec{m_local_imu_data.gyro_radps.value() - m_bias.gyro};
   }

   void publish_data()
   {
      const volatile uint32_t clock = ClockSource::now_ms();
      m_imu_data_storage.update_latest(m_local_imu_data, clock);

      m_data_log_counter++;
      if ((m_data_log_counter % 250u) == 0)
      {
         m_logger.printf("clock: %u   accel x: %.2f, y: %.2f, z: %.2f     |     gyro: x: %.2f, y: %.2f, z: %.2f     |     temp: %.4f",
                         clock,
                         m_local_imu_data.accel_mps2.value().x,
                         m_local_imu_data.accel_mps2.value().y,
                         m_local_imu_data.accel_mps2.value().z,
                         m_local_imu_data.gyro_radps.value().x,
                         m_local_imu_data.gyro_radps.value().y,
                         m_local_imu_data.gyro_radps.value().z,
                         m_local_imu_data.temperature_c.value());
      }
   }

   void publish_health()
   {
      m_imu_health_storage.update_latest(m_local_imu_health, ClockSource::now_ms());
   }

   void reset_data()
   {
      m_local_imu_data.accel_mps2.reset();
      m_local_imu_data.gyro_radps.reset();
      m_local_imu_data.temperature_c.reset();

      m_imu_data_storage.update_latest(m_local_imu_data, ClockSource::now_ms());
   }

   void mark_validation_fail()
   {
      m_local_imu_health.validation_ok = false;
      m_logger.print("validation failed");
   }

   void mark_validation_success()
   {
      m_local_imu_health.validation_ok = true;
      m_logger.print("validation successful");
   }

   void mark_self_test_fail()
   {
      m_local_imu_health.self_test_ok = false;
      m_logger.print("self-test failed");
   }

   void mark_self_test_pass()
   {
      m_local_imu_health.self_test_ok = true;
      m_logger.print("self-test successful");
   }

   void mark_config_fail()
   {
      m_local_imu_health.config_ok = false;
      m_logger.print("config failed");
   }

   void mark_config_success()
   {
      m_local_imu_health.config_ok = true;
      m_logger.print("config successful");
   }

   void set_state(const imu_sensor::ImuSensorState& state)
   {
      m_local_imu_health.state = state;

      switch (m_local_imu_health.state)
      {
         case imu_sensor::ImuSensorState::stopped:
            m_logger.print("entered state->stopped");
            break;
         case imu_sensor::ImuSensorState::reset:
            m_logger.print("entered state->reset");
            break;
         case imu_sensor::ImuSensorState::validation:
            m_logger.print("entered state->validation");
            break;
         case imu_sensor::ImuSensorState::self_test:
            m_logger.print("entered state->self_test");
            break;
         case imu_sensor::ImuSensorState::config:
            m_logger.print("entered state->config");
            break;
         case imu_sensor::ImuSensorState::operational:
            m_logger.print("entered state->operational");
            break;
         case imu_sensor::ImuSensorState::soft_recovery:
            m_logger.print("entered state->soft_recovery");
            break;
         case imu_sensor::ImuSensorState::hard_recovery:
            m_logger.print("entered state->hard_recovery");
            break;
         case imu_sensor::ImuSensorState::failure:
            m_logger.print("entered state->failure");
            break;
         default:
            m_logger.print("entered unexpected state");
            error::stop_operation();
            break;
      }
   }

   void set_error(const imu_sensor::ImuSensorError& error)
   {
      m_local_imu_health.error.set(static_cast<uint8_t>(error));

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
         case imu_sensor::ImuSensorError::unstable_gyro_error:
            m_logger.print("encountered error->unstable_gyro_error");
            break;
         case imu_sensor::ImuSensorError::unstable_accel_error:
            m_logger.print("encountered error->unstable_accel_error");
            break;
         case imu_sensor::ImuSensorError::max_error:
         default:
            m_logger.print("encountered unexpected error");
            error::stop_operation();
            break;
      }
   }

   void set_unstable_samples_error()
   {
      if (!is_gravity_reasonable() || !is_accel_stable())
      {
         set_error(imu_sensor::ImuSensorError::unstable_accel_error);
      }

      if (!is_gyro_stable())
      {
         set_error(imu_sensor::ImuSensorError::unstable_gyro_error);
      }
   }

   imu_sensor::ImuSensorState get_state() const
   {
      return m_local_imu_health.state;
   }

   imu_sensor::ErrorBits get_error() const
   {
      return m_local_imu_health.error;
   }

   bool validation_successful() const
   {
      return m_local_imu_health.validation_ok;
   }

   bool self_test_successful() const
   {
      return m_local_imu_health.self_test_ok;
   }

   bool config_successful() const
   {
      return m_local_imu_health.config_ok;
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

   bool is_sensor_sane() const
   {
      return (is_gravity_reasonable() && is_accel_stable() && is_gyro_stable());
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
      return (m_local_imu_health.read_failure_count < m_read_failures_limit);
   }

   bool all_samples_collected() const
   {
      return (m_self_test_stats.sample_counter >= m_num_samples_self_test);
   }

private:
   void calculate_standard_deviation()
   {
      error::verify(m_self_test_stats.sample_counter > 1u);

      // finalize welford's algorithm
      const float n_minus_one = static_cast<float>(m_self_test_stats.sample_counter) - 1.0f;

      const Vec variance_accel = m_self_test_stats.ssq_accel / n_minus_one;
      const Vec variance_gyro  = m_self_test_stats.ssq_gyro / n_minus_one;

      const float std_accel_x = std::sqrt(variance_accel.x);
      const float std_accel_y = std::sqrt(variance_accel.y);
      const float std_accel_z = std::sqrt(variance_accel.z);
      const float std_gyro_x  = std::sqrt(variance_gyro.x);
      const float std_gyro_y  = std::sqrt(variance_gyro.y);
      const float std_gyro_z  = std::sqrt(variance_gyro.z);

      m_self_test_stats.std_accel = Vec{std_accel_x, std_accel_y, std_accel_z};
      m_self_test_stats.std_gyro  = Vec{std_gyro_x, std_gyro_y, std_gyro_z};
   }

   void calculate_bias()
   {
      const float magnitude = get_norm(m_self_test_stats.mean_accel);
      error::verify(magnitude > 0);

      // unit vector -> a^ = a/|a|
      // gravity_body vector -> G_b = G * a^
      const Vec gravity_body = {m_self_test_stats.mean_accel.x * g_to_mps2 / magnitude,
                                m_self_test_stats.mean_accel.y * g_to_mps2 / magnitude,
                                m_self_test_stats.mean_accel.z * g_to_mps2 / magnitude};

      // bias = a - G_b
      // accel bias is the residual vector after subtracting the G component
      m_bias.accel.x = m_self_test_stats.mean_accel.x - gravity_body.x;
      m_bias.accel.y = m_self_test_stats.mean_accel.y - gravity_body.y;
      m_bias.accel.z = m_self_test_stats.mean_accel.z - gravity_body.z;

      // gyro bias is just mean
      m_bias.gyro = m_self_test_stats.mean_gyro;
   }

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

   static constexpr float get_norm(const Vec& vec) noexcept
   {
      return std::sqrt((vec.x * vec.x) + (vec.y * vec.y) + (vec.z * vec.z));
   }

   bool is_gyro_range_plausible() const
   {
      // sensor range check -> must be performed with raw/uncalibrated data
      error::verify(m_local_imu_data.gyro_radps.has_value());
      const auto& g = m_local_imu_data.gyro_radps.value();
      return ((std::fabs(g.x) <= m_gyro_absolute_plausibility_limit_radps) &&
              (std::fabs(g.y) <= m_gyro_absolute_plausibility_limit_radps) &&
              (std::fabs(g.z) <= m_gyro_absolute_plausibility_limit_radps));
   }

   bool is_accel_range_plausible() const
   {
      // sensor range check -> must be performed with raw/uncalibrated data
      error::verify(m_local_imu_data.accel_mps2.has_value());
      const auto& a = m_local_imu_data.accel_mps2.value();
      return ((std::fabs(a.x) <= m_accel_absolute_plausibility_limit_mps2) &&
              (std::fabs(a.y) <= m_accel_absolute_plausibility_limit_mps2) &&
              (std::fabs(a.z) <= m_accel_absolute_plausibility_limit_mps2));
   }

   bool is_temp_plausible() const
   {
      // sensor range check -> must be performed with raw/uncalibrated data
      if (!m_local_imu_data.temperature_c.has_value())
      {
         return true;   // nothing to check
      }

      return ((params::temp_min_plauisble_range < m_local_imu_data.temperature_c.value()) &&
              (m_local_imu_data.temperature_c.value() < params::temp_max_plauisble_range));
   }

   bool is_gravity_reasonable() const
   {
      return std::abs(get_norm(m_self_test_stats.mean_accel) - g_to_mps2) <= m_accel_tolerance_mps2;
   }

   bool is_accel_stable() const
   {
      return ((m_self_test_stats.std_accel.x <= m_accel_tolerance_mps2) &&
              (m_self_test_stats.std_accel.y <= m_accel_tolerance_mps2) &&
              (m_self_test_stats.std_accel.z <= m_accel_tolerance_mps2));
   }

   bool is_gyro_stable() const
   {
      return ((m_self_test_stats.std_gyro.x <= m_gyro_tolerance_radps) &&
              (m_self_test_stats.std_gyro.y <= m_gyro_tolerance_radps) &&
              (m_self_test_stats.std_gyro.z <= m_gyro_tolerance_radps));
   }

   struct Config_registers
   {
      uint8_t sample_rate_div;
      uint8_t config;
      uint8_t gyro_config;
      uint8_t accel_config;
      uint8_t accel_config_2;
   };

   struct SelfTestStats
   {
      // mean, (ssq) aggregation of squared distances from the mean and (std) standard deviation
      Vec      mean_accel, ssq_accel, std_accel;
      Vec      mean_gyro, ssq_gyro, std_gyro;
      uint32_t sample_counter{};   // N
   };

   struct Bias
   {
      Vec accel{};
      Vec gyro{};
   };

   static constexpr uint8_t one_byte_shift  = 8u;
   static constexpr uint8_t three_bit_shift = 3u;
   static constexpr float   deg_to_rad      = 3.14159265358979323846f / 180.0f;
   static constexpr float   g_to_mps2       = 9.80665f;

   ImuData&                                           m_imu_data_storage;
   ImuHealth&                                         m_imu_health_storage;
   SpiMaster&                                         m_spi_master;
   Logger&                                            m_logger;
   const uint8_t                                      m_read_failures_limit;
   const std::size_t                                  m_execution_period_ms;
   const std::size_t                                  m_receive_wait_timeout_ms;
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
   const uint8_t                                      m_num_samples_self_test;
   const float                                        m_gyro_tolerance_radps;
   const float                                        m_accel_tolerance_mps2;
   std::array<uint8_t, params::num_bytes_transaction> m_tx_buffer{};
   std::array<uint8_t, params::num_bytes_transaction> m_rx_buffer{};
   imu_sensor::ImuData                                m_local_imu_data{};
   imu_sensor::ImuHealth                              m_local_imu_health{};
   SelfTestStats                                      m_self_test_stats{};
   Bias                                               m_bias{};
   uint8_t                                            m_device_id{};
   Config_registers                                   m_config_registers{};
   std::size_t                                        m_wait_timer_ms{};
   std::size_t                                        m_data_log_counter{};
};

}   // namespace mpu6500
