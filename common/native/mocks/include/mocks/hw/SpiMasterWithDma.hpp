#pragma once

#include "error/error_handler.hpp"

namespace mocks::hw
{

template <std::size_t N>
class SpiMasterWithDma
{
   using TransferCompleteCallback = void (*)(void*);

public:
   void transfer(std::span<const uint8_t> tx_buffer, std::span<uint8_t> rx_buffer)
   {
      copy_buffer(m_tx_buffer, tx_buffer);
      copy_buffer(rx_buffer, m_rx_buffer);
   }

   void register_transfer_complete_callback(const TransferCompleteCallback callback, void* context)
   {
      error::verify(callback != nullptr);
      m_transfer_complete_callback = callback;
      m_callback_context           = context;
   }

   static void copy_buffer(auto& src, auto& dest)
   {
      std::copy(dest.begin(), dest.end(), src.begin());
   }

   std::array<uint8_t, N>   m_tx_buffer{};
   std::array<uint8_t, N>   m_rx_buffer{};
   TransferCompleteCallback m_transfer_complete_callback{nullptr};
   void*                    m_callback_context{nullptr};
};

}   // namespace mocks::hw
