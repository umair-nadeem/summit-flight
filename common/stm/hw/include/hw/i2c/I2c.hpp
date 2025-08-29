#pragma once

#include "I2cConfig.hpp"

namespace hw::i2c
{

class I2c
{
public:
   explicit I2c(I2cConfig& config)
       : m_i2c_config{config}
   {
   }

   void prepare_for_communication()
   {
      re_init_12c();
   }

   void read()
   {
      rc = bmp390_read_reg_1byte_ll(0x76u, 0x00u, &id);
      if (rc == 0)
      {
         /* success: id contains chip id (BMP390 typical value = 0x60) */
         // toggle an LED here or send id over UART
      }
      else
      {
         /* rc < 0 indicates reason — you can inspect/print rc for debugging */
      }
   }

   // bool read(uint16_t slave_address, std::span<uint8_t> buffer)
   // {
   //    re_init_12c();
   //    error::verify(buffer.size() <= maxBytesPerI2cTransfer);

   //    LL_I2C_HandleTransfer(m_i2c_config.i2c_handle,
   //                          (static_cast<uint32_t>(slave_address) << 1u),
   //                          LL_I2C_ADDRSLAVE_7BIT,
   //                          buffer.size(),
   //                          LL_I2C_MODE_AUTOEND,
   //                          LL_I2C_GENERATE_START_READ);

   //    size_t rxByteIdx = 0;
   //    while (LL_I2C_IsActiveFlag_STOP(m_i2c_config.i2c_handle) == 0u)   // Loop until end of transfer received
   //    {
   //       if (LL_I2C_IsActiveFlag_ARLO(m_i2c_config.i2c_handle) == 1u)
   //       {
   //          re_init_12c();
   //          return false;   // arbitration lost
   //       }

   //       if (LL_I2C_IsActiveFlag_TCR(m_i2c_config.i2c_handle) == 1u)
   //       {
   //          break;   // maximum number of bytes transmitted, or transmission complete
   //       }

   //       if (LL_I2C_IsActiveFlag_RXNE(m_i2c_config.i2c_handle) == 1u)
   //       {
   //          buffer[rxByteIdx] = LL_I2C_ReceiveData8(m_i2c_config.i2c_handle);
   //          rxByteIdx++;
   //       }
   //    }
   //    re_init_12c();
   //    return true;
   // }

private:
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
   int bmp390_read_reg_1byte_ll(uint8_t addr7, uint8_t reg, uint8_t* out)
   {
      uint32_t timeout;
      uint8_t  rx;

      /* --- PHASE 1: MASTER WRITE (send register address) --- */

      LL_I2C_GenerateStartCondition(m_i2c_config.i2c_handle);

      /* wait for SB (start generated) */
      timeout = 100000U;
      while (!LL_I2C_IsActiveFlag_SB(m_i2c_config.i2c_handle))
         if (--timeout == 0U)
            return -1;

      /* send control byte: 7-bit address + write(0) */
      LL_I2C_TransmitData8(m_i2c_config.i2c_handle, static_cast<uint8_t>(addr7 << 1));

      /* wait for ADDR (address sent & acked) */
      timeout = 100000U;
      while (!LL_I2C_IsActiveFlag_ADDR(m_i2c_config.i2c_handle))
      {
         if (--timeout == 0U)
         {
            /* possible NACK or bus error */
            if (LL_I2C_IsActiveFlag_AF(m_i2c_config.i2c_handle))
               LL_I2C_ClearFlag_AF(m_i2c_config.i2c_handle);
            return -2;
         }
      }
      /* clear ADDR flag by reading SR1 and SR2 (LL helper) */
      LL_I2C_ClearFlag_ADDR(m_i2c_config.i2c_handle);

      /* wait TXE (data register empty) */
      timeout = 100000U;
      while (!LL_I2C_IsActiveFlag_TXE(m_i2c_config.i2c_handle))
         if (--timeout == 0U)
            return -3;

      /* transmit register index */
      LL_I2C_TransmitData8(m_i2c_config.i2c_handle, reg);

      /* wait for byte transfer finished (BTF) or TXE */
      timeout = 100000U;
      while (!LL_I2C_IsActiveFlag_BTF(m_i2c_config.i2c_handle))
         if (--timeout == 0U)
            break;

      /* --- PHASE 2: MASTER RESTART + READ 1 BYTE --- */

      LL_I2C_GenerateStartCondition(m_i2c_config.i2c_handle);

      timeout = 100000U;
      while (!LL_I2C_IsActiveFlag_SB(m_i2c_config.i2c_handle))
         if (--timeout == 0U)
            return -4;

      /* send control byte: 7-bit address + read(1) */
      LL_I2C_TransmitData8(m_i2c_config.i2c_handle, (static_cast<uint8_t>(addr7 << 1u) | 1u));

      /* wait for ADDR (address + read acked) */
      timeout = 100000U;
      while (!LL_I2C_IsActiveFlag_ADDR(m_i2c_config.i2c_handle))
      {
         if (--timeout == 0U)
         {
            if (LL_I2C_IsActiveFlag_AF(m_i2c_config.i2c_handle))
               LL_I2C_ClearFlag_AF(m_i2c_config.i2c_handle);
            return -5;
         }
      }

      /* For a single byte read, tell peripheral to NACK the byte and generate STOP
         BEFORE clearing ADDR in order to meet the STM32 read sequence for N=1. */
      LL_I2C_AcknowledgeNextData(m_i2c_config.i2c_handle, LL_I2C_NACK);
      LL_I2C_ClearFlag_ADDR(m_i2c_config.i2c_handle);        /* clear ADDR */
      LL_I2C_GenerateStopCondition(m_i2c_config.i2c_handle); /* generate STOP after this byte */

      /* wait for RXNE then read data */
      timeout = 100000U;
      while (!LL_I2C_IsActiveFlag_RXNE(m_i2c_config.i2c_handle))
         if (--timeout == 0U)
            return -6;

      rx = LL_I2C_ReceiveData8(m_i2c_config.i2c_handle);

      /* clear STOP flag if set (safe) */
      if (LL_I2C_IsActiveFlag_STOP(m_i2c_config.i2c_handle))
         LL_I2C_ClearFlag_STOP(m_i2c_config.i2c_handle);

      *out = rx;
      return 0;
   }

   I2cConfig& m_i2c_config;
   uint8_t    id{};
   int        rc{};
};

}   // namespace hw::i2c
