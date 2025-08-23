#pragma once

#include <span>

#include "Mpu6500Error.hpp"
#include "Mpu6500State.hpp"
#include "boundaries/SensorData.hpp"
#include "error/error_handler.hpp"
#include "imu_sensor/ImuData.hpp"
#include "interfaces/IClockSource.hpp"
#include "params.hpp"

namespace mpu6500
{

template <interfaces::IClockSource ClockSource, typename SpiMaster>
class Mpu6500StateHandler
{
   using ImuData = ::boundaries::SensorData<imu_sensor::ImuData>;

public:
   explicit Mpu6500StateHandler(ImuData&          imu_data_storage,
                                SpiMaster&        spi_master,
                                const std::size_t execution_period_ms,
                                const std::size_t receive_wait_timeout_ms)
       : m_imu_data_storage{imu_data_storage},
         m_spi_master{spi_master},
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
      m_tx_buffer[1] = params::sample_rate_divider;   // sample rate
      m_tx_buffer[2] = params::dlpf_config;           // config reg
      m_tx_buffer[3] = params::gyro_full_scale;       // gyro config reg
      m_tx_buffer[4] = params::accel_full_scale;      // accel config reg
      m_tx_buffer[5] = params::accel_a_dlpf_config;   // accel config 2 reg
      m_spi_master.transfer(std::span{m_tx_buffer.data(), burst_len}, std::span{m_rx_buffer.data(), burst_len});
   }

   void read_config_burst()
   {
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

   void store_id()
   {
      m_device_id = m_rx_buffer[1];
   }

   void store_config()
   {
      m_config_registers.sample_rate_div = m_rx_buffer[1];
      m_config_registers.config          = static_cast<uint8_t>(m_rx_buffer[2] & params::ConfigBitMask::dlpf_cfg);
      m_config_registers.gyro_config     = static_cast<uint8_t>(m_rx_buffer[3] & params::GyroConfigBitMask::gyro_fs_sel);
      m_config_registers.accel_config    = static_cast<uint8_t>(m_rx_buffer[4] & params::AccelConfigBitMask::accel_fs_sel);
      m_config_registers.accel_config_2  = static_cast<uint8_t>(m_rx_buffer[5] & params::AccelConfig2BitMask::accel_a_dlpf_config);
   }

   void mark_validation_fail()
   {
      m_validation_successful = false;
   }

   void mark_validation_success()
   {
      m_validation_successful = true;
   }

   void mark_self_test_fail()
   {
      m_self_test_successful = false;
   }

   void mark_self_test_pass()
   {
      m_self_test_successful = true;
   }

   void mark_config_fail()
   {
      m_config_successful = false;
   }

   void mark_config_success()
   {
      m_config_successful = true;
   }

   void set_state(const Mpu6500State& state)
   {
      m_state = state;
   }

   void set_error(const Mpu6500Error& error)
   {
      m_error = error;
   }

   Mpu6500State get_state() const
   {
      return m_state;
   }

   Mpu6500Error get_error() const
   {
      return m_error;
   }

   bool validation_successful() const
   {
      return m_validation_successful;
   }

   bool self_test_successful() const
   {
      return m_self_test_successful;
   }

   bool config_successful() const
   {
      return m_config_successful;
   }

   bool id_matched() const
   {
      return (m_device_id == params::device_id);
   }

   bool config_matched() const
   {
      return ((m_config_registers.sample_rate_div == params::sample_rate_divider) &&
              (m_config_registers.config == params::dlpf_config) &&
              (m_config_registers.gyro_config == params::gyro_full_scale) &&
              (m_config_registers.accel_config == params::accel_full_scale) &&
              (m_config_registers.accel_config_2 == params::accel_a_dlpf_config));
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

private:
   // void callback()
   // {
   //    read_raw_data_from_rx_buffer();
   //    m_imu_data_storage.update_latest(m_local_imu_data, ClockSource::now_ms());
   //    m_spi_rx_done.store(true, std::memory_order_release);
   // }

   // inline void read_raw_data_from_rx_buffer()
   // {
   //    m_local_imu_data.accel_mps2.x = to_int16(m_rx_buffer[1], m_rx_buffer[2]) * ACCEL_SCALE;
   //    m_local_imu_data.accel_mps2.y = to_int16(m_rx_buffer[3], m_rx_buffer[4]) * ACCEL_SCALE;
   //    m_local_imu_data.accel_mps2.z = to_int16(m_rx_buffer[5], m_rx_buffer[6]) * ACCEL_SCALE;

   //    m_local_imu_data.temperature_c = to_int16(m_rx_buffer[7], m_rx_buffer[8]) * TEMP_SCALE + TEMP_OFFSET;

   //    m_local_imu_data.gyro_radps.x = to_int16(m_rx_buffer[9], m_rx_buffer[10]) * GYRO_SCALE;
   //    m_local_imu_data.gyro_radps.y = to_int16(m_rx_buffer[11], m_rx_buffer[12]) * GYRO_SCALE;
   //    m_local_imu_data.gyro_radps.z = to_int16(m_rx_buffer[13], m_rx_buffer[14]) * GYRO_SCALE;
   // }

   static constexpr uint8_t get_write_spi_command(const uint8_t reg) noexcept
   {
      return (reg & static_cast<uint8_t>(~params::read_mask));
   }

   static constexpr uint8_t get_read_spi_command(const uint8_t reg) noexcept
   {
      return (reg | params::read_mask);
   }

   // static constexpr auto to_int16(uint8_t msb, uint8_t lsb) noexcept
   // {
   //    return static_cast<int16_t>((msb << 8) | lsb);
   // }

   struct config_registers
   {
      uint8_t sample_rate_div;
      uint8_t config;
      uint8_t gyro_config;
      uint8_t accel_config;
      uint8_t accel_config_2;
   };

   ImuData&                                           m_imu_data_storage;
   SpiMaster&                                         m_spi_master;
   const std::size_t                                  m_execution_period_ms;
   const std::size_t                                  m_receive_wait_timeout_ms;
   std::array<uint8_t, params::num_bytes_transaction> m_tx_buffer{};
   std::array<uint8_t, params::num_bytes_transaction> m_rx_buffer{};
   imu_sensor::ImuData                                m_local_imu_data{};
   Mpu6500State                                       m_state{Mpu6500State::stopped};
   Mpu6500Error                                       m_error{Mpu6500Error::none};
   uint8_t                                            m_device_id{};
   config_registers                                   m_config_registers{};
   std::atomic<bool>                                  m_spi_rx_done{false};
   bool                                               m_validation_successful{false};
   bool                                               m_self_test_successful{false};
   bool                                               m_config_successful{false};
   std::size_t                                        m_wait_timer_ms{};

   static constexpr float ACCEL_SCALE = 9.81f / 16384.0f;
   static constexpr float GYRO_SCALE  = 3.14159f / (180.0f * 131.0f);
   static constexpr float TEMP_SCALE  = 1.0f / 340.0f;
   static constexpr float TEMP_OFFSET = 36.53f;
};

}   // namespace mpu6500
