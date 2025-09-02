#pragma once

#include <optional>
#include <span>

#include "I2cConfig.hpp"
#include "error/error_handler.hpp"

namespace hw::i2c
{

template <typename TimeoutEvaluator>
class I2c
{
public:
   explicit I2c(I2cConfig& config, TimeoutEvaluator& timeout_evaluator)
       : m_i2c_config{config},
         m_timeout_evaluator{timeout_evaluator}
   {
   }

   void prepare_for_communication()
   {
      re_init_12c();
   }

   // Write buffer to slave
   bool write(const uint8_t i2c_addr, std::span<const uint8_t> tx_buffer, uint32_t timeout = 100000)
   {
      const auto length = tx_buffer.size();
      error::verify(length <= max_bytes_per_i2c_transfer);

      // generate start condition
      if (!start(i2c_addr, false, timeout))
      {
         return false;
      }

      // send data
      if (!send_bytes(tx_buffer, timeout))
      {
         return false;
      }

      stop();
      return true;
   }

   // Read into buffer from slave
   bool read(const uint8_t i2c_addr, std::optional<uint8_t> reg, std::span<uint8_t> rx_buffer, uint32_t timeout = 100000)
   {
      const auto length = rx_buffer.size();
      error::verify(length <= max_bytes_per_i2c_transfer);

      if (reg.has_value())
      {
         std::array<uint8_t, 1u> reg_addr = {reg.value()};
         // generate start condition for sending register address
         if (!start(i2c_addr, false, timeout))
         {
            return false;
         }

         // send register
         if (!send_bytes(reg_addr, timeout))
         {
            return false;
         }
      }

      // generate re-start condition for reading data
      if (!start(i2c_addr, true, timeout))
      {
         return false;
      }

      // receive data
      if (!receive_bytes(rx_buffer, timeout))
      {
         return false;
      }

      stop();
      return true;
   }

private:
   bool start(const uint8_t i2c_addr, const bool read, const uint32_t timeout)
   {
      uint32_t tick = 0;

      // Generate START
      LL_I2C_GenerateStartCondition(m_i2c_config.i2c_handle);
      while (!LL_I2C_IsActiveFlag_SB(m_i2c_config.i2c_handle))
      {
         if (tick++ > timeout)
         {
            re_init_12c();
            return false;
         }
      }

      // Send address
      if (read)
      {
         LL_I2C_TransmitData8(m_i2c_config.i2c_handle, static_cast<uint8_t>(i2c_addr << 1u) | 0x01);
      }
      else
      {
         LL_I2C_TransmitData8(m_i2c_config.i2c_handle, static_cast<uint8_t>(i2c_addr << 1u) & ~0x01);
      }

      tick = 0;
      while (!LL_I2C_IsActiveFlag_ADDR(m_i2c_config.i2c_handle))
      {
         if (tick++ > timeout)
         {
            re_init_12c();
            return false;
         }
      }

      LL_I2C_ClearFlag_ADDR(m_i2c_config.i2c_handle);

      return true;
   }

   void stop()
   {
      LL_I2C_GenerateStopCondition(m_i2c_config.i2c_handle);
   }

   bool send_bytes(std::span<const uint8_t> tx_buffer, const uint32_t timeout)
   {
      const auto  length = tx_buffer.size();
      std::size_t index  = 0;
      uint32_t    tick   = 0;

      while (index < length)
      {
         if (tick++ > timeout)
         {
            re_init_12c();
            return false;
         }

         if (LL_I2C_IsActiveFlag_TXE(m_i2c_config.i2c_handle) == 1u)
         {
            LL_I2C_TransmitData8(m_i2c_config.i2c_handle, tx_buffer[index]);
            ++index;
         }
      }

      return true;
   }

   bool receive_bytes(std::span<uint8_t> rx_buffer, const uint32_t timeout)
   {
      const auto  length = rx_buffer.size();
      std::size_t index  = 0;
      uint32_t    tick   = 0;

      while (index < length)
      {
         if (tick++ > timeout)
         {
            re_init_12c();
            return false;
         }

         if (index == length - 1u)
         {
            // last byte: disable ACK, generate STOP
            LL_I2C_AcknowledgeNextData(m_i2c_config.i2c_handle, LL_I2C_NACK);
         }
         else
         {
            LL_I2C_AcknowledgeNextData(m_i2c_config.i2c_handle, LL_I2C_ACK);
         }

         if (LL_I2C_IsActiveFlag_RXNE(m_i2c_config.i2c_handle) == 1u)
         {
            rx_buffer[index] = LL_I2C_ReceiveData8(m_i2c_config.i2c_handle);
            ++index;
         }
      }

      return true;
   }

   void re_init_12c()
   {
      while ((LL_I2C_IsActiveFlag_STOP(m_i2c_config.i2c_handle) == 1u) ||
             (LL_I2C_IsActiveFlag_ARLO(m_i2c_config.i2c_handle) == 1u))   // Loop until end of transfer received
      {
         LL_I2C_Disable(m_i2c_config.i2c_handle);
         LL_I2C_ClearFlag_STOP(m_i2c_config.i2c_handle);
         LL_I2C_ClearFlag_ARLO(m_i2c_config.i2c_handle);
      }

      while (LL_I2C_IsEnabled(m_i2c_config.i2c_handle) == 0u)
      {
         LL_I2C_Enable(m_i2c_config.i2c_handle);
      }
   }

   /* Read 1 byte register (blocking), returns 0 on success, negative error on failure.
      addr7: 7-bit I2C address (e.g. BMP390_ADDR0 or BMP390_ADDR1)
      out: pointer to return byte */
   // int bmp390_read_reg_1byte_ll(uint8_t addr7, uint8_t reg)
   // {
   //    uint32_t timeout;
   //    uint8_t  rx;

   //    /* --- PHASE 1: MASTER WRITE (send register address) --- */

   //    LL_I2C_GenerateStartCondition(m_i2c_config.i2c_handle);

   //    /* wait for SB (start generated) */
   //    timeout = 100000U;
   //    while (!LL_I2C_IsActiveFlag_SB(m_i2c_config.i2c_handle))
   //       if (--timeout == 0U)
   //          return -1;

   //    /* send control byte: 7-bit address + write(0) */
   //    LL_I2C_TransmitData8(m_i2c_config.i2c_handle, static_cast<uint8_t>(addr7 << 1));

   //    /* wait for ADDR (address sent & acked) */
   //    timeout = 100000U;
   //    while (!LL_I2C_IsActiveFlag_ADDR(m_i2c_config.i2c_handle))
   //    {
   //       if (--timeout == 0U)
   //       {
   //          /* possible NACK or bus error */
   //          if (LL_I2C_IsActiveFlag_AF(m_i2c_config.i2c_handle))
   //             LL_I2C_ClearFlag_AF(m_i2c_config.i2c_handle);
   //          return -2;
   //       }
   //    }
   //    /* clear ADDR flag by reading SR1 and SR2 (LL helper) */
   //    LL_I2C_ClearFlag_ADDR(m_i2c_config.i2c_handle);

   //    /* wait TXE (data register empty) */
   //    timeout = 100000U;
   //    while (!LL_I2C_IsActiveFlag_TXE(m_i2c_config.i2c_handle))
   //       if (--timeout == 0U)
   //          return -3;

   //    /* transmit register index */
   //    LL_I2C_TransmitData8(m_i2c_config.i2c_handle, reg);

   //    /* wait for byte transfer finished (BTF) or TXE */
   //    timeout = 100000U;
   //    while (!LL_I2C_IsActiveFlag_BTF(m_i2c_config.i2c_handle))
   //       if (--timeout == 0U)
   //          break;

   //    /* --- PHASE 2: MASTER RESTART + READ 1 BYTE --- */

   //    LL_I2C_GenerateStartCondition(m_i2c_config.i2c_handle);

   //    timeout = 100000U;
   //    while (!LL_I2C_IsActiveFlag_SB(m_i2c_config.i2c_handle))
   //       if (--timeout == 0U)
   //          return -4;

   //    /* send control byte: 7-bit address + read(1) */
   //    LL_I2C_TransmitData8(m_i2c_config.i2c_handle, (static_cast<uint8_t>(addr7 << 1u) | 1u));

   //    /* wait for ADDR (address + read acked) */
   //    timeout = 100000U;
   //    while (!LL_I2C_IsActiveFlag_ADDR(m_i2c_config.i2c_handle))
   //    {
   //       if (--timeout == 0U)
   //       {
   //          if (LL_I2C_IsActiveFlag_AF(m_i2c_config.i2c_handle))
   //             LL_I2C_ClearFlag_AF(m_i2c_config.i2c_handle);
   //          return -5;
   //       }
   //    }

   //    /* For a single byte read, tell peripheral to NACK the byte and generate STOP
   //       BEFORE clearing ADDR in order to meet the STM32 read sequence for N=1. */
   //    LL_I2C_AcknowledgeNextData(m_i2c_config.i2c_handle, LL_I2C_NACK);
   //    LL_I2C_ClearFlag_ADDR(m_i2c_config.i2c_handle);        /* clear ADDR */
   //    LL_I2C_GenerateStopCondition(m_i2c_config.i2c_handle); /* generate STOP after this byte */

   //    /* wait for RXNE then read data */
   //    timeout = 100000U;
   //    while (!LL_I2C_IsActiveFlag_RXNE(m_i2c_config.i2c_handle))
   //       if (--timeout == 0U)
   //          return -6;

   //    rx = LL_I2C_ReceiveData8(m_i2c_config.i2c_handle);

   //    /* clear STOP flag if set (safe) */
   //    if (LL_I2C_IsActiveFlag_STOP(m_i2c_config.i2c_handle))
   //       LL_I2C_ClearFlag_STOP(m_i2c_config.i2c_handle);

   //    id = rx;
   //    return 0;
   // }

   static constexpr std::size_t max_bytes_per_i2c_transfer = 255u;

   I2cConfig&        m_i2c_config;
   TimeoutEvaluator& m_timeout_evaluator;
   uint8_t           id{};
};

}   // namespace hw::i2c
