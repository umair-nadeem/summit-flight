#pragma once

#include <atomic>
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
      while ((LL_I2C_IsActiveFlag_STOP(m_i2c_config.i2c_handle) == 1u) ||
             (LL_I2C_IsActiveFlag_ARLO(m_i2c_config.i2c_handle) == 1u))   // Loop until end of transfer received
      {
         LL_I2C_Disable(m_i2c_config.i2c_handle);
         clear_status();
      }

      while (LL_I2C_IsEnabled(m_i2c_config.i2c_handle) == 0u)
      {
         LL_I2C_Enable(m_i2c_config.i2c_handle);
      }

      m_state = I2cState::idle;
   }

   bool attempt_reinit()
   {
      bool success = false;

      if ((LL_I2C_IsActiveFlag_STOP(m_i2c_config.i2c_handle) == 1u) ||
          (LL_I2C_IsActiveFlag_ARLO(m_i2c_config.i2c_handle) == 1u))
      {
         LL_I2C_Disable(m_i2c_config.i2c_handle);
         clear_status();
      }

      if (LL_I2C_IsEnabled(m_i2c_config.i2c_handle) == 0u)
      {
         LL_I2C_Enable(m_i2c_config.i2c_handle);
         success = true;
         m_state = I2cState::idle;
      }

      return success;
   }

   // Start write transaction (non-blocking)
   bool write(const uint8_t i2c_addr, std::span<const uint8_t> tx_buffer)
   {
      error::verify(tx_buffer.size() <= max_bytes_per_i2c_transfer);

      if (m_state == I2cState::idle)
      {
         m_current_i2c_addr = i2c_addr;
         m_tx_buffer        = tx_buffer;
         m_rx_buffer        = {};
         m_read_reg         = std::nullopt;
         m_tx_index         = 0;
         m_rx_index         = 0;

         enable_interrupts();

         m_read_transaction.store(false, std::memory_order_relaxed);
         m_state = I2cState::start_condition_for_data;

         // generate start
         LL_I2C_GenerateStartCondition(m_i2c_config.i2c_handle);

         return true;
      }

      return false;
   }

   // Start read transaction (non-blocking)
   bool read(const uint8_t i2c_addr, std::span<uint8_t> rx_buffer, std::optional<uint8_t> reg)
   {
      error::verify(rx_buffer.size() <= max_bytes_per_i2c_transfer);

      if (m_state == I2cState::idle)
      {
         m_current_i2c_addr = i2c_addr;
         m_tx_buffer        = {};
         m_rx_buffer        = rx_buffer;
         m_read_reg         = reg;
         m_tx_index         = 0;
         m_rx_index         = 0;

         enable_interrupts();

         m_read_transaction.store(true, std::memory_order_relaxed);

         if (m_read_reg.has_value())
         {
            m_state = I2cState::start_condition_for_read_register;
         }
         else
         {
            m_state = I2cState::start_condition_for_data;
         }

         // generate start
         LL_I2C_GenerateStartCondition(m_i2c_config.i2c_handle);
         return true;
      }

      return false;
   }

   void handle_i2c_event_interrupt()
   {
      const auto state            = m_state;
      const auto read_transaction = m_read_transaction.load(std::memory_order_relaxed);

      switch (state)
      {
         case I2cState::idle:
         case I2cState::error:
            // do nothing
            break;

         case I2cState::start_condition_for_read_register:
            if (LL_I2C_IsActiveFlag_SB(m_i2c_config.i2c_handle) == 1u)
            {
               LL_I2C_TransmitData8(m_i2c_config.i2c_handle, static_cast<uint8_t>(m_current_i2c_addr << 1u) & ~0x01);
               m_state = I2cState::addr_flag_wait_for_read_register;
            }
            break;

         case I2cState::addr_flag_wait_for_read_register:
            if (LL_I2C_IsActiveFlag_ADDR(m_i2c_config.i2c_handle) == 1u)
            {
               LL_I2C_ClearFlag_ADDR(m_i2c_config.i2c_handle);
               m_state = I2cState::send_read_register;
            }
            break;

         case I2cState::send_read_register:
            if (LL_I2C_IsActiveFlag_TXE(m_i2c_config.i2c_handle) == 1u)
            {
               LL_I2C_TransmitData8(m_i2c_config.i2c_handle, m_read_reg.value());
               LL_I2C_GenerateStartCondition(m_i2c_config.i2c_handle);   // re-generate start condition
               m_state = I2cState::start_condition_for_data;
            }
            break;

         case I2cState::start_condition_for_data:
            if (LL_I2C_IsActiveFlag_SB(m_i2c_config.i2c_handle) == 1u)
            {
               if (read_transaction)
               {
                  LL_I2C_TransmitData8(m_i2c_config.i2c_handle, static_cast<uint8_t>(m_current_i2c_addr << 1u) | 0x01);
               }
               else
               {
                  LL_I2C_TransmitData8(m_i2c_config.i2c_handle, static_cast<uint8_t>(m_current_i2c_addr << 1u) & ~0x01);
               }

               m_state = I2cState::addr_flag_wait_for_data;
            }
            break;

         case I2cState::addr_flag_wait_for_data:
            if (LL_I2C_IsActiveFlag_ADDR(m_i2c_config.i2c_handle) == 1u)
            {
               if (read_transaction)
               {
                  LL_I2C_ClearFlag_ADDR(m_i2c_config.i2c_handle);

                  // for single-byte read, NACK is made before ADDR is cleared
                  if (m_rx_buffer.size() == 1u)
                  {
                     LL_I2C_AcknowledgeNextData(m_i2c_config.i2c_handle, LL_I2C_NACK);
                     LL_I2C_GenerateStopCondition(m_i2c_config.i2c_handle);
                  }
                  else
                  {
                     LL_I2C_AcknowledgeNextData(m_i2c_config.i2c_handle, LL_I2C_ACK);
                  }
               }
               m_state = I2cState::data_transaction;
            }
            break;

         case I2cState::data_transaction:
            if (read_transaction)
            {
               const auto rx_length = m_rx_buffer.size();
               if (m_rx_index < rx_length)
               {
                  receive_byte();
                  // the ACK bit must be cleared just after reading the second last data byte
                  if (m_rx_index == rx_length - 1u)
                  {
                     LL_I2C_AcknowledgeNextData(m_i2c_config.i2c_handle, LL_I2C_NACK);
                     LL_I2C_GenerateStopCondition(m_i2c_config.i2c_handle);
                  }
                  else
                  {
                     LL_I2C_AcknowledgeNextData(m_i2c_config.i2c_handle, LL_I2C_ACK);
                  }
               }

               if (m_rx_index == rx_length)
               {
                  stop(true);   // rx data read complete
               }
            }
            else
            {
               // write
               const auto tx_length = m_tx_buffer.size();
               if (m_tx_index < tx_length)
               {
                  send_byte();
               }

               if (m_tx_index == tx_length)
               {
                  stop(true);   // tx data write complete
               }
            }
            break;

         default:
            error::stop_operation();
            break;
      }
   }

   void handle_i2c_error_interrupt()
   {
      stop(false);
   }

   bool has_error() const
   {
      return (m_state == I2cState::error);
   }

private:
   void stop(const bool success)
   {
      clear_status();

      // clear any spurious byte lingering around
      if (LL_I2C_IsActiveFlag_RXNE(m_i2c_config.i2c_handle) == 1u)
      {
         [[maybe_unused]] auto _ = LL_I2C_ReceiveData8(m_i2c_config.i2c_handle);
      }

      disable_interrupts();

      if (success)
      {
         m_state = I2cState::idle;
      }
      else
      {
         m_state = I2cState::error;
      }
   }

   void receive_byte()
   {
      if (LL_I2C_IsActiveFlag_RXNE(m_i2c_config.i2c_handle) == 1u)
      {
         m_rx_buffer[m_rx_index] = LL_I2C_ReceiveData8(m_i2c_config.i2c_handle);
         ++m_rx_index;
      }
   }

   void send_byte()
   {
      if (LL_I2C_IsActiveFlag_TXE(m_i2c_config.i2c_handle) == 1u)
      {
         LL_I2C_TransmitData8(m_i2c_config.i2c_handle, m_tx_buffer[m_tx_index]);
         ++m_tx_index;
      }
   }

   void enable_interrupts()
   {
      LL_I2C_EnableIT_EVT(m_i2c_config.i2c_handle);
      LL_I2C_EnableIT_ERR(m_i2c_config.i2c_handle);
      LL_I2C_EnableIT_BUF(m_i2c_config.i2c_handle);
   }

   void disable_interrupts()
   {
      LL_I2C_DisableIT_EVT(m_i2c_config.i2c_handle);
      LL_I2C_DisableIT_ERR(m_i2c_config.i2c_handle);
      LL_I2C_DisableIT_BUF(m_i2c_config.i2c_handle);
   }

   void clear_status()
   {
      LL_I2C_ClearFlag_STOP(m_i2c_config.i2c_handle);
      LL_I2C_ClearFlag_ARLO(m_i2c_config.i2c_handle);
      LL_I2C_ClearFlag_BERR(m_i2c_config.i2c_handle);
      LL_I2C_ClearFlag_OVR(m_i2c_config.i2c_handle);
      LL_I2C_ClearFlag_AF(m_i2c_config.i2c_handle);
   }

   static constexpr std::size_t max_bytes_per_i2c_transfer = 255u;

   enum class I2cState : uint32_t
   {
      idle = 0,
      start_condition_for_read_register,
      addr_flag_wait_for_read_register,
      send_read_register,
      start_condition_for_data,
      addr_flag_wait_for_data,
      data_transaction,
      error
   };

   // transaction context
   I2cConfig&               m_i2c_config;
   uint8_t                  m_current_i2c_addr{0};
   std::span<const uint8_t> m_tx_buffer{};
   std::span<uint8_t>       m_rx_buffer{};
   std::optional<uint8_t>   m_read_reg{};
   size_t                   m_tx_index{0};
   size_t                   m_rx_index{0};
   volatile I2cState        m_state{I2cState::idle};
   std::atomic<bool>        m_read_transaction{};
};

}   // namespace hw::i2c
