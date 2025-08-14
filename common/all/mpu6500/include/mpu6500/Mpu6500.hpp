#pragma once

#include <boost/sml.hpp>

#include "Mpu6500StateHandler.hpp"
#include "boundaries/SensorData.hpp"
#include "imu_sensor/ImuData.hpp"
#include "interfaces/IClockSource.hpp"

namespace mpu6500
{

template <interfaces::IClockSource ClockSource, typename SpiMaster>
class Mpu6500
{
   using ImuDataStorage = ::boundaries::SensorData<imu_sensor::ImuData, ClockSource>;

   // When using SPI interface, user should use PWR_MGMT_1 (register 107) as well as
   // SIGNAL_PATH_RESET (register 104) to ensure the reset is performed properly. The sequence
   //  used should be:
   // Set H_RESET = 1 (register PWR_MGMT_1)
   // Wait 100ms
   // Set GYRO_RST = ACCEL_RST = TEMP_RST = 1 (register SIGNAL_PATH_RESET)
   // Wait 100ms
   // verify whoami
   // clock source setup (auto 1 instead of internal)
   // self-test (only once)
   // configuration of gyro and accel
   // configure ranges GYRO = ±2000 dps, ACCEL = ±8 g
   // sample rate setup
   //    DLPF: enable (choose DLPF_CFG such as 3 or 4 depending on bandwidth needed — see datasheet table).
   //  Example DLPF_CFG=3 (~44 Hz) often used for flight control smoothing. overall filter cutoff freq adds delay
   // SMPLRT_DIV: set so sample rate = 250 Hz (SMPLRT_DIV = 3 when DLPF enabled).

   //////////////////////////////////////////////////////////////////////////////////

   //  State: RESET_START
   //   - write PWR_MGMT_1: set DEVICE_RESET bit (H_RESET = 1)
   //   - wait 100 ms
   //   - next -> SIGNAL_PATH_RESET

   // State: SIGNAL_PATH_RESET
   //   - write SIGNAL_PATH_RESET: set GYRO_RST = 1, ACCEL_RST = 1, TEMP_RST = 1
   //   - wait 100 ms
   //   - next -> WHOAMI_CHECK

   // State: WHOAMI_CHECK
   //   - read WHO_AM_I (register)
   //   - if value == 0x70 -> next -> CLK_AND_WAKE
   //   - else retry N times (e.g. 3) with small delay; on repeated failure -> FAULT

   // State: CLK_AND_WAKE
   //   - write PWR_MGMT_1 to select clock and wake device:
   //       e.g. clear SLEEP bit, set CLKSEL to PLL/auto (CLKSEL = 1 or explicit PLL_X)
   //       (also clear DEVICE_RESET bits if necessary)
   //   - small delay (10 ms)
   //   - optional: read back PWR_MGMT_1 to verify written value
   //   - next -> (optional) SELF_TEST or CONFIG_RANGES

   // Optional State: SELF_TEST (recommended for ground checks)
   //   - read baseline accel/gyro values (normal mode)
   //   - enable self-test bits: write ACCEL_CONFIG and GYRO_CONFIG self-test bits (per datasheet)
   //   - wait short settling time (e.g. 20–50 ms)
   //   - read accel/gyro outputs with self-test enabled
   //   - compute self-test response = (ST_output - baseline)
   //   - read factory self-test / trim registers (SELF_TEST / ST_OTP) from device
   //   - compute pass/fail per datasheet criteria (ratio for gyro, bounds or ratio for accel)
   //   - if pass -> next -> CONFIGURE_SENSORS
   //   - else -> depending on policy: retry or FAULT (recommended fail to safe)

   // State: CONFIGURE_SENSORS
   //   - write GYRO_CONFIG to set FS_SEL (full scale) and to clear self-test bits
   //   - write ACCEL_CONFIG to set AFS_SEL and clear self-test bits
   //   - write CONFIG (DLPF_CFG) to set low-pass filter bandwidth
   //   - write ACCEL_CONFIG2 (if needed) for accel DLPF
   //   - verify by reading back the registers

   // State: SET_SAMPLE_RATE
   //   - decide sensor ODR. For a 4 ms loop (250 Hz) recommended:
   //       * enable DLPF (so gyro output rate = 1 kHz)
   //       * SMPLRT_DIV = 1000/250 - 1 = 3  -> Sample Rate = 1000 / (1 + 3) = 250 Hz
   //   - write SMPLRT_DIV = 3
   //   - consider enabling DRDY interrupt if you want interrupt-driven reads (set INT_ENABLE register)
   //   - next -> OPERATIONAL

   // State: OPERATIONAL
   //   - every 4 ms (or on DRDY interrupt) read sensor data:
   //       * burst read the accel, temp, gyro registers in a single SPI transaction
   //       * validate data (not all zeros, no stale counts, sequence checks)
   //   - if repeated SPI errors or invalid data -> attempt limited recovery:
   //       * try SIGNAL_PATH_RESET -> reconfigure -> if still failing -> FAULT

   // State: FAULT
   //   - record fault (with reason)
   //   - try to reinitialize after backoff or report to higher-level mission controller

public:
   explicit Mpu6500(ImuDataStorage& data_storage, SpiMaster& spi_master)
       : m_data_storage{data_storage},
         m_spi_master{spi_master}
   {
      m_spi_master.register_transfer_complete_callback(&Mpu6500::spi_isr_trampoline, this);
   }

   void execute()
   {
      process_states();
   }

   // This is called from ISR via SPI driver (must be ISR-safe)
   void spi_transfer_complete_callback()
   {
      read_raw_data_from_rx_buffer();
      m_data_storage.update_latest(m_imu_data);
      m_spi_rx_done.store(true, std::memory_order_release);
   }

private:
   template <typename StateHandler>
   struct Mpu6500StateMachine
   {

      struct e1
      {
      };
      struct e2
      {
      };
      struct e3
      {
      };
      struct e4
      {
      };
      struct e5
      {
      };

      auto operator()() const
      {
         using namespace boost::sml;

         auto guard1 = []
         {
            return true;
         };

         auto action1 = [](auto e) {};

         // clang-format off
         return make_transition_table(
            *"idle"_s + event<e1> = "s1"_s
            , "s1"_s + event<e2> [ guard1 ] / action1 = "s2"_s
         );
         // clang-format on
      }
   };

   enum class Mpu6500State
   {
      reset,
      wakeup,
      verify_id,
      read_data,
      done,
      error
   };

   void process_states()
   {
      switch (m_state)
      {
         case Mpu6500State::reset:
            m_tx_buffer[0] = 0x6B & 0x7F;
            m_tx_buffer[1] = 0x00;
            m_spi_master.transfer(std::span{m_tx_buffer.data(), 2u}, std::span{m_rx_buffer.data(), 2u});

            m_state = Mpu6500State::wakeup;
            break;

         case Mpu6500State::wakeup:
            m_tx_buffer[0] = 0x75 | 0x80;   // read mode
            m_tx_buffer[1] = 0x00;          // dummy
            m_spi_master.transfer(std::span{m_tx_buffer.data(), 2u}, std::span{m_rx_buffer.data(), 2u});

            m_state = Mpu6500State::verify_id;
            break;

         case Mpu6500State::verify_id:
            if (m_rx_buffer[1] == 0x70)
            {
               m_state = Mpu6500State::read_data;
            }
            else
            {
               m_state = Mpu6500State::error;
            }
            break;

         case Mpu6500State::read_data:
            // Read 14 bytes starting at ACCEL_XOUT_H (0x3B)
            m_tx_buffer[0] = 0x3B | 0x80;                                   // read mode
            std::fill(m_tx_buffer.begin() + 1u, m_tx_buffer.end(), 0x00);   // dummy bytes

            m_spi_master.transfer(std::span{m_tx_buffer.data(), 15u}, std::span{m_rx_buffer.data(), 15u});

            // rx_buffer[1]..rx_buffer[14] now contain accel, temp, gyro
            m_state = Mpu6500State::done;
            break;

         case Mpu6500State::done:
            break;

         case Mpu6500State::error:
            break;

         default:
            error::stop_operation();
            break;
      }
   }

   // Static trampoline to call the instance method
   static void spi_isr_trampoline(void* ctx)
   {
      auto* self = static_cast<Mpu6500*>(ctx);
      self->spi_transfer_complete_callback();
   }

   static constexpr auto to_int16(uint8_t msb, uint8_t lsb)
   {
      return static_cast<int16_t>((msb << 8) | lsb);
   }

   inline void read_raw_data_from_rx_buffer()
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
   Mpu6500State                                       m_state{Mpu6500State::reset};

   static constexpr float ACCEL_SCALE = 9.81f / 16384.0f;
   static constexpr float GYRO_SCALE  = 3.14159f / (180.0f * 131.0f);
   static constexpr float TEMP_SCALE  = 1.0f / 340.0f;
   static constexpr float TEMP_OFFSET = 36.53f;
};

}   // namespace mpu6500
