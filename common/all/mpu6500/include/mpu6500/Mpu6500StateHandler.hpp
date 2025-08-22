#pragma once

#include <span>

#include "Mpu6500State.hpp"
#include "boundaries/SensorData.hpp"
#include "error/error_handler.hpp"
#include "imu_sensor/ImuData.hpp"
#include "params.hpp"

namespace mpu6500
{

template <typename SpiMaster>
class Mpu6500StateHandler
{
   using ImuData = ::boundaries::SensorData<imu_sensor::ImuData>;

public:
   explicit Mpu6500StateHandler(ImuData& imu_data_storage, SpiMaster& spi_master)
       : m_imu_data_storage{imu_data_storage},
         m_spi_master{spi_master}
   {
   }

   // void tick_timer()
   // {
   // }

   // void reset_timer()
   // {
   // }

   // void power_reset()
   // {
   // }

   // void signal_path_reset()
   // {
   // }

   // void set_bus()
   // {
   // }

   // void set_clock_and_wakeup()
   // {
   // }

   // void read_id()
   // {
   // }

   // void verify_id()
   // {
   // }

   // void mark_validation_fail()
   // {
   // }

   // void mark_validation_success()
   // {
   // }

   void set_state(const Mpu6500State& state)
   {
      m_state = state;
   }

   // bool id_matched()
   // {
   //    return false;
   // }

   // bool power_reset_wait_over()
   // {
   //    return false;
   // }

   // bool signal_reset_wait_over()
   // {
   //    return false;
   // }

   // void process_states()
   {
      // switch (m_state)
      // {
      //    case Mpu6500State::reset:
      //       m_tx_buffer[0] = 0x6B & 0x7F;
      //       m_tx_buffer[1] = 0x00;
      //       m_spi_master.transfer(std::span{m_tx_buffer.data(), 2u}, std::span{m_rx_buffer.data(), 2u});

      //       m_state = Mpu6500State::wakeup;
      //       break;

      //    case Mpu6500State::wakeup:
      //       m_tx_buffer[0] = 0x75 | 0x80;   // read mode
      //       m_tx_buffer[1] = 0x00;          // dummy
      //       m_spi_master.transfer(std::span{m_tx_buffer.data(), 2u}, std::span{m_rx_buffer.data(), 2u});

      //       m_state = Mpu6500State::verify_id;
      //       break;

      //    case Mpu6500State::verify_id:
      //       if (m_rx_buffer[1] == 0x70)
      //       {
      //          m_state = Mpu6500State::read_data;
      //       }
      //       else
      //       {
      //          m_state = Mpu6500State::error;
      //       }
      //       break;

      //    case Mpu6500State::read_data:
      //       // Read 14 bytes starting at ACCEL_XOUT_H (0x3B)
      //       m_tx_buffer[0] = 0x3B | 0x80;                                   // read mode
      //       std::fill(m_tx_buffer.begin() + 1u, m_tx_buffer.end(), 0x00);   // dummy bytes

      //       m_spi_master.transfer(std::span{m_tx_buffer.data(), 15u}, std::span{m_rx_buffer.data(), 15u});

      //       // rx_buffer[1]..rx_buffer[14] now contain accel, temp, gyro
      //       m_state = Mpu6500State::done;
      //       break;

      //    case Mpu6500State::done:
      //       break;

      //    case Mpu6500State::error:
      //       break;

      //    default:
      //       error::stop_operation();
      //       break;
      // }
   }

   void callback()
   {
      read_raw_data_from_rx_buffer();

      m_spi_rx_done.store(true, std::memory_order_release);
   }

   inline void read_raw_data_from_rx_buffer()
   {
      m_local_imu_data.accel_mps2.x = to_int16(m_rx_buffer[1], m_rx_buffer[2]) * ACCEL_SCALE;
      m_local_imu_data.accel_mps2.y = to_int16(m_rx_buffer[3], m_rx_buffer[4]) * ACCEL_SCALE;
      m_local_imu_data.accel_mps2.z = to_int16(m_rx_buffer[5], m_rx_buffer[6]) * ACCEL_SCALE;

      m_local_imu_data.temperature_c = to_int16(m_rx_buffer[7], m_rx_buffer[8]) * TEMP_SCALE + TEMP_OFFSET;

      m_local_imu_data.gyro_radps.x = to_int16(m_rx_buffer[9], m_rx_buffer[10]) * GYRO_SCALE;
      m_local_imu_data.gyro_radps.y = to_int16(m_rx_buffer[11], m_rx_buffer[12]) * GYRO_SCALE;
      m_local_imu_data.gyro_radps.z = to_int16(m_rx_buffer[13], m_rx_buffer[14]) * GYRO_SCALE;
   }

private:
   static constexpr auto to_int16(uint8_t msb, uint8_t lsb) noexcept
   {
      return static_cast<int16_t>((msb << 8) | lsb);
   }

   ImuData&                                           m_imu_data_storage;
   SpiMaster&                                         m_spi_master;
   std::array<uint8_t, params::num_bytes_transaction> m_tx_buffer{};
   std::array<uint8_t, params::num_bytes_transaction> m_rx_buffer{};
   imu_sensor::ImuData                                m_local_imu_data{};
   std::atomic<bool>                                  m_spi_rx_done{false};
   Mpu6500State                                       m_state{Mpu6500State::stopped};

   static constexpr float ACCEL_SCALE = 9.81f / 16384.0f;
   static constexpr float GYRO_SCALE  = 3.14159f / (180.0f * 131.0f);
   static constexpr float TEMP_SCALE  = 1.0f / 340.0f;
   static constexpr float TEMP_OFFSET = 36.53f;
};

}   // namespace mpu6500
