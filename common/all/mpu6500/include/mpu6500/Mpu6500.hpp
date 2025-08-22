#pragma once

#include "MainSM.hpp"
#include "Mpu6500StateHandler.hpp"

namespace mpu6500
{

// When using SPI interface, user should use PWR_MGMT_1 (register 107) as well as
// SIGNAL_PATH_RESET (register 104) to ensure the reset is performed properly. The sequence
//  used should be:
// Set H
// Wait 1_RESET = 1 (register PWR_MGMT_1)00ms
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
template <interfaces::IClockSource ClockSource, typename SpiMaster>
class Mpu6500
{
   using ImuData = ::boundaries::SensorData<imu_sensor::ImuData>;

public:
   explicit Mpu6500(ImuData&          imu_data_storage,
                    SpiMaster&        spi_master,
                    const std::size_t execution_period_ms,
                    const std::size_t receive_wait_timeout_ms)
       : m_state_handler{imu_data_storage, spi_master, execution_period_ms, receive_wait_timeout_ms}
   {
      spi_master.register_transfer_complete_callback(&Mpu6500::spi_isr_trampoline, this);
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

   // This is called from ISR via SPI driver (must be ISR-safe)
   void spi_transfer_complete_callback()
   {
      m_state_machine.process_event(EventReceiveDone{});
   }

   Mpu6500State get_state() const
   {
      return m_state_handler.get_state();
   }

   Mpu6500Error get_error() const
   {
      return m_state_handler.get_error();
   }

private:
   // Static trampoline to call the instance method
   static void spi_isr_trampoline(void* ctx)
   {
      auto* self = static_cast<Mpu6500*>(ctx);
      self->spi_transfer_complete_callback();
   }

   using StateHandler    = Mpu6500StateHandler<ClockSource, SpiMaster>;
   using StateMachineDef = MainStateMachine<StateHandler>;

   StateHandler                    m_state_handler;
   boost::sml::sm<StateMachineDef> m_state_machine{m_state_handler};
};

}   // namespace mpu6500
