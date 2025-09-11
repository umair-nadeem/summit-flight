#pragma once

#include "error/error_handler.hpp"

namespace mocks::hw
{

class I2c
{
   using TransferCompleteCallback = void (*)(void*);

public:
   bool write(const uint8_t i2c_addr, std::span<const uint8_t> tx_buffer)
   {
      m_i2c_addr = i2c_addr;
      copy_buffer(tx_buffer, m_tx_buffer);
      m_tx_buffer_span = std::span{m_tx_buffer.data(), tx_buffer.size()};
      return m_transaction_result;
   }

   bool read(const uint8_t i2c_addr, std::span<uint8_t> rx_buffer, std::optional<uint8_t> reg)
   {
      m_i2c_addr = i2c_addr;
      m_read_reg = reg;
      copy_buffer(m_rx_buffer_span, rx_buffer);
      return m_transaction_result;
   }

   void register_transfer_complete_callback(const TransferCompleteCallback callback, void* context)
   {
      error::verify(callback != nullptr);
      m_transfer_complete_callback = callback;
      m_callback_context           = context;
   }

   void stage_rx_buffer(std::span<const uint8_t> staging_rx_buffer)
   {
      copy_buffer(staging_rx_buffer, m_rx_buffer);
      m_rx_buffer_span = std::span{m_rx_buffer.data(), staging_rx_buffer.size()};
   }

   static void copy_buffer(std::span<const uint8_t> src, std::span<uint8_t> dest)
   {
      error::verify(src.size() <= dest.size());
      std::copy(src.begin(), src.end(), dest.begin());
   }

   static constexpr std::size_t max_bytes_per_i2c_transfer = 255u;

   std::array<uint8_t, max_bytes_per_i2c_transfer> m_tx_buffer{};
   std::array<uint8_t, max_bytes_per_i2c_transfer> m_rx_buffer{};
   std::span<uint8_t>                              m_tx_buffer_span{};
   std::span<uint8_t>                              m_rx_buffer_span{};
   uint8_t                                         m_i2c_addr{};
   std::optional<uint8_t>                          m_read_reg{};
   bool                                            m_transaction_result{true};
   TransferCompleteCallback                        m_transfer_complete_callback{nullptr};
   void*                                           m_callback_context{nullptr};
};

}   // namespace mocks::hw
