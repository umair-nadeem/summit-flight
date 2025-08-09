#include "hw/dma/dma.hpp"

#include "error/error_handler.hpp"

namespace hw::dma
{

void clear_dma_tc_flag(DMA_TypeDef* dma_handle, const uint32_t dma_stream)
{
   switch (dma_stream)
   {
      case LL_DMA_STREAM_0:
         LL_DMA_ClearFlag_TC0(dma_handle);
         break;

      case LL_DMA_STREAM_1:
         LL_DMA_ClearFlag_TC1(dma_handle);
         break;

      case LL_DMA_STREAM_2:
         LL_DMA_ClearFlag_TC2(dma_handle);
         break;

      case LL_DMA_STREAM_3:
         LL_DMA_ClearFlag_TC3(dma_handle);
         break;

      case LL_DMA_STREAM_4:
         LL_DMA_ClearFlag_TC4(dma_handle);
         break;

      case LL_DMA_STREAM_5:
         LL_DMA_ClearFlag_TC5(dma_handle);
         break;

      case LL_DMA_STREAM_6:
         LL_DMA_ClearFlag_TC6(dma_handle);
         break;

      case LL_DMA_STREAM_7:
         LL_DMA_ClearFlag_TC7(dma_handle);
         break;

      default:
         error::stop_operation();
         break;
   }
}

void clear_dma_te_flag(DMA_TypeDef* dma_handle, const uint32_t dma_stream)
{
   switch (dma_stream)
   {
      case LL_DMA_STREAM_0:
         LL_DMA_ClearFlag_TE0(dma_handle);
         break;

      case LL_DMA_STREAM_1:
         LL_DMA_ClearFlag_TE1(dma_handle);
         break;

      case LL_DMA_STREAM_2:
         LL_DMA_ClearFlag_TE2(dma_handle);
         break;

      case LL_DMA_STREAM_3:
         LL_DMA_ClearFlag_TE3(dma_handle);
         break;

      case LL_DMA_STREAM_4:
         LL_DMA_ClearFlag_TE4(dma_handle);
         break;

      case LL_DMA_STREAM_5:
         LL_DMA_ClearFlag_TE5(dma_handle);
         break;

      case LL_DMA_STREAM_6:
         LL_DMA_ClearFlag_TE6(dma_handle);
         break;

      case LL_DMA_STREAM_7:
         LL_DMA_ClearFlag_TE7(dma_handle);
         break;

      default:
         error::stop_operation();
         break;
   }
}

bool is_dma_tc_flag_active(DMA_TypeDef* dma_handle, const uint32_t dma_stream)
{
   switch (dma_stream)
   {
      case LL_DMA_STREAM_0:
         return (LL_DMA_IsActiveFlag_TC0(dma_handle) == 1u);

      case LL_DMA_STREAM_1:
         return (LL_DMA_IsActiveFlag_TC1(dma_handle) == 1u);

      case LL_DMA_STREAM_2:
         return (LL_DMA_IsActiveFlag_TC2(dma_handle) == 1u);

      case LL_DMA_STREAM_3:
         return (LL_DMA_IsActiveFlag_TC3(dma_handle) == 1u);

      case LL_DMA_STREAM_4:
         return (LL_DMA_IsActiveFlag_TC4(dma_handle) == 1u);

      case LL_DMA_STREAM_5:
         return (LL_DMA_IsActiveFlag_TC5(dma_handle) == 1u);

      case LL_DMA_STREAM_6:
         return (LL_DMA_IsActiveFlag_TC6(dma_handle) == 1u);

      case LL_DMA_STREAM_7:
         return (LL_DMA_IsActiveFlag_TC7(dma_handle) == 1u);

      default:
         error::stop_operation();
         break;
   }

   return false;
}

}   // namespace hw::dma