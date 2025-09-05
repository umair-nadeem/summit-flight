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
   using ReceiveCompleteCallback = void (*)(void*);

public:
   explicit I2c(I2cConfig& config);

   void prepare_for_communication();

   bool attempt_reinit();

   // Start write transaction (non-blocking)
   bool write(const uint8_t i2c_addr, std::span<const uint8_t> tx_buffer);

   // Start read transaction (non-blocking)
   bool read(const uint8_t i2c_addr, std::span<uint8_t> rx_buffer, std::optional<uint8_t> reg);

   void handle_i2c_event_interrupt();

   void handle_i2c_error_interrupt();

   bool has_error() const;

   void register_receive_complete_callback(const ReceiveCompleteCallback callback, void* context);

private:
   void stop(const bool success);

   void receive_byte();

   void send_byte();

   void enable_interrupts();

   void disable_interrupts();

   void clear_status();

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
   ReceiveCompleteCallback  m_receive_complete_callback{nullptr};
   void*                    m_callback_context{nullptr};
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
