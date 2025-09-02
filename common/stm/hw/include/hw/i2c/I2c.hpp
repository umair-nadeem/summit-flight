#pragma once

#include <optional>
#include <span>

#include "I2cConfig.hpp"
#include "error/error_handler.hpp"

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

   static constexpr std::size_t max_bytes_per_i2c_transfer = 255u;

   I2cConfig& m_i2c_config;
};

}   // namespace hw::i2c
