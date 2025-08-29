#pragma once

#include <atomic>
#include <span>

#include "error/error_handler.hpp"
#include "hw/dma/dma.hpp"
#include "hw/spi/SpiWithDmaConfig.hpp"
#include "interfaces/pcb_component/IEnabler.hpp"

namespace hw::spi
{

template <interfaces::pcb_component::IEnabler ChipSelect>
class SpiMasterWithDma
{
   using TransferCompleteCallback = void (*)(void*);

public:
   explicit SpiMasterWithDma(SpiWithDmaConfig& spi_config, ChipSelect& chip_select)
       : m_spi_config{spi_config},
         m_chip_select{chip_select}
   {
   }

   void prepare_for_communication()
   {
      LL_DMA_EnableIT_TC(m_spi_config.dma_handle, m_spi_config.rx_dma_stream);

      LL_SPI_Enable(m_spi_config.spi_handle);
   }

   void transfer(std::span<const uint8_t> tx_buffer, std::span<uint8_t> rx_buffer)
   {
      error::verify(!m_transfer_in_progress.load(std::memory_order_acquire));
      error::verify(tx_buffer.data() != rx_buffer.data());
      error::verify(tx_buffer.size() == rx_buffer.size());
      error::verify(!tx_buffer.empty());
      m_transfer_in_progress.store(true, std::memory_order_release);

      LL_SPI_EnableDMAReq_RX(m_spi_config.spi_handle);
      LL_SPI_EnableDMAReq_TX(m_spi_config.spi_handle);

      LL_DMA_DisableStream(m_spi_config.dma_handle, m_spi_config.rx_dma_stream);
      LL_DMA_DisableStream(m_spi_config.dma_handle, m_spi_config.tx_dma_stream);

      // clear stale flags
      clear_stale_flags();

      LL_DMA_ConfigAddresses(m_spi_config.dma_handle,
                             m_spi_config.rx_dma_stream,
                             LL_SPI_DMA_GetRegAddr(m_spi_config.spi_handle),
                             reinterpret_cast<uint32_t>(rx_buffer.data()),
                             LL_DMA_DIRECTION_PERIPH_TO_MEMORY);

      LL_DMA_ConfigAddresses(m_spi_config.dma_handle,
                             m_spi_config.tx_dma_stream,
                             reinterpret_cast<uint32_t>(tx_buffer.data()),
                             LL_SPI_DMA_GetRegAddr(m_spi_config.spi_handle),
                             LL_DMA_DIRECTION_MEMORY_TO_PERIPH);

      LL_DMA_SetDataLength(m_spi_config.dma_handle, m_spi_config.rx_dma_stream, rx_buffer.size());
      LL_DMA_SetDataLength(m_spi_config.dma_handle, m_spi_config.tx_dma_stream, tx_buffer.size());

      enable_chip_select();

      LL_DMA_EnableStream(m_spi_config.dma_handle, m_spi_config.rx_dma_stream);
      LL_DMA_EnableStream(m_spi_config.dma_handle, m_spi_config.tx_dma_stream);
   }

   void register_transfer_complete_callback(const TransferCompleteCallback callback, void* context)
   {
      error::verify(callback != nullptr);
      m_transfer_complete_callback = callback;
      m_callback_context           = context;
   }

   void handle_spi_transfer_complete_interrupt()
   {
      if (::hw::dma::is_dma_tc_flag_active(m_spi_config.dma_handle, m_spi_config.rx_dma_stream))
      {
         ::hw::dma::clear_dma_tc_flag(m_spi_config.dma_handle, m_spi_config.rx_dma_stream);

         LL_SPI_DisableDMAReq_RX(m_spi_config.spi_handle);
         LL_SPI_DisableDMAReq_TX(m_spi_config.spi_handle);

         disable_chip_select();

         m_transfer_in_progress.store(false, std::memory_order_release);

         if (m_transfer_complete_callback != nullptr)
         {
            m_transfer_complete_callback(m_callback_context);
         }
      }
   }

private:
   void enable_chip_select()
   {
      m_chip_select.enable();
   }

   void disable_chip_select()
   {
      m_chip_select.disable();
   }

   void clear_stale_flags()
   {
      ::hw::dma::clear_dma_tc_flag(m_spi_config.dma_handle, m_spi_config.rx_dma_stream);
      ::hw::dma::clear_dma_tc_flag(m_spi_config.dma_handle, m_spi_config.tx_dma_stream);

      ::hw::dma::clear_dma_te_flag(m_spi_config.dma_handle, m_spi_config.rx_dma_stream);
      ::hw::dma::clear_dma_te_flag(m_spi_config.dma_handle, m_spi_config.tx_dma_stream);
   }

   SpiWithDmaConfig&        m_spi_config;
   ChipSelect&              m_chip_select;
   TransferCompleteCallback m_transfer_complete_callback{nullptr};
   void*                    m_callback_context{nullptr};
   std::atomic<bool>        m_transfer_in_progress{false};
};

}   // namespace hw::spi
