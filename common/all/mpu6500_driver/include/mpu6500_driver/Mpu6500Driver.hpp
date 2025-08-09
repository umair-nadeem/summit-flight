#pragma once

#include <functional>

#include "boundaries/SensorDataStorage.hpp"
#include "imu_sensor/ImuData.hpp"
#include "interfaces/IClockSource.hpp"
#include "mpu6500_params.hpp"

namespace mpu6500_driver
{

template <interfaces::IClockSource ClockSource, typename SpiMaster>
class Mpu6500Driver
{
   using ImuDataStorage = ::boundaries::SensorDataStorage<imu_sensor::ImuData, ClockSource>;

public:
   explicit Mpu6500Driver(ImuDataStorage& data_storage, SpiMaster& spi_master)
       : m_data_storage{data_storage},
         m_spi_master{spi_master}
   {
      m_spi_master.register_transfer_complete_callback(&Mpu6500Driver::spi_isr_trampoline, this);
   }

   void execute()
   {
      // state machine to do init sensor and read data
      m_spi_master.transfer(m_tx_buffer, m_rx_buffer);
   }

   void read_data()
   {
      m_spi_master.transfer(m_tx_buffer, m_rx_buffer);
   }

   // This is called from ISR via SPI driver (must be ISR-safe)
   void spi_transfer_complete_callback()
   {
      read_raw_from_rx_buffer();
      m_data_storage.update_latest(m_imu_data);
      m_spi_rx_done.store(true, std::memory_order_release);
   }

   // Static trampoline to call the instance method
   static void spi_isr_trampoline(void* ctx)
   {
      auto* self = static_cast<Mpu6500Driver*>(ctx);
      self->spi_transfer_complete_callback();
   }

private:
   static constexpr auto to_int16(uint8_t msb, uint8_t lsb)
   {
      return static_cast<int16_t>((msb << 8) | lsb);
   }

   inline void read_raw_from_rx_buffer()
   {
      m_imu_data.accel_mps2.x = to_int16(m_rx_buffer[1], m_rx_buffer[2]) * ACCEL_SCALE;
      m_imu_data.accel_mps2.y = to_int16(m_rx_buffer[3], m_rx_buffer[4]) * ACCEL_SCALE;
      m_imu_data.accel_mps2.z = to_int16(m_rx_buffer[5], m_rx_buffer[6]) * ACCEL_SCALE;

      m_imu_data.temperature_c = to_int16(m_rx_buffer[7], m_rx_buffer[8]) * TEMP_SCALE + TEMP_OFFSET;

      m_imu_data.gyro_radps.x = to_int16(m_rx_buffer[9], m_rx_buffer[10]) * GYRO_SCALE;
      m_imu_data.gyro_radps.y = to_int16(m_rx_buffer[11], m_rx_buffer[12]) * GYRO_SCALE;
      m_imu_data.gyro_radps.z = to_int16(m_rx_buffer[13], m_rx_buffer[14]) * GYRO_SCALE;
   }

   ImuDataStorage&                                    m_data_storage;
   SpiMaster&                                         m_spi_master;
   std::array<uint8_t, params::num_bytes_transaction> m_tx_buffer{};
   std::array<uint8_t, params::num_bytes_transaction> m_rx_buffer{};
   imu_sensor::ImuData                                m_imu_data{};
   std::atomic<bool>                                  m_spi_rx_done{false};

   static constexpr float ACCEL_SCALE = 9.81f / 16384.0f;
   static constexpr float GYRO_SCALE  = 3.14159f / (180.0f * 131.0f);
   static constexpr float TEMP_SCALE  = 1.0f / 340.0f;
   static constexpr float TEMP_OFFSET = 36.53f;
};

}   // namespace mpu6500_driver
