#pragma once

#include "error/error_handler.hpp"

namespace mocks::hw
{

class SpiMasterWithDma
{
   using TransferCompleteCallback = void (*)(void*);

public:
   SpiMasterWithDma(std::span<uint8_t> tx_buffer, std::span<uint8_t> rx_buffer)
       : m_tx_buffer(tx_buffer),
         m_rx_buffer(rx_buffer)
   {
   }

   void transfer(std::span<const uint8_t> tx_buffer, std::span<uint8_t> rx_buffer)
   {
      copy_buffer(tx_buffer, m_tx_buffer);
      copy_buffer(std::span{m_rx_buffer.data(), rx_buffer.size()}, rx_buffer);
   }

   void register_transfer_complete_callback(const TransferCompleteCallback callback, void* context)
   {
      error::verify(callback != nullptr);
      m_transfer_complete_callback = callback;
      m_callback_context           = context;
   }

   static void copy_buffer(std::span<const uint8_t> src, std::span<uint8_t> dest)
   {
      error::verify(src.size() <= dest.size());
      std::copy(src.begin(), src.end(), dest.begin());
   }

   std::span<uint8_t>       m_tx_buffer;
   std::span<uint8_t>       m_rx_buffer;
   TransferCompleteCallback m_transfer_complete_callback{nullptr};
   void*                    m_callback_context{nullptr};
};

}   // namespace mocks::hw
