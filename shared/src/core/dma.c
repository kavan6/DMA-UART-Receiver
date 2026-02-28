#include <libopencm3/stm32/dma.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/usart.h>

#include "core/dma.h"

#define BUFFER_LENGTH   64

#define USING_DMA       (DMA1)
#define USING_STREAM    (DMA_STREAM5)
#define USING_CHANNEL   (DMA_SxCR_CHSEL_4)

static uint8_t buffer[BUFFER_LENGTH];

static uint16_t read_pos = 0;

static uint16_t dma_get_write_pos(void)
{
    return BUFFER_LENGTH - dma_get_number_of_data(DMA1, DMA_STREAM5);
}

static void dma_read_buffer(uint8_t* data, uint16_t pos)
{
    *data = buffer[pos];

    read_pos++;
    if (read_pos >= BUFFER_LENGTH)
    {
        read_pos = 0;
    }
}

bool dma_data_available(void)
{
    return dma_get_write_pos() != read_pos;
}

bool dma_read(uint8_t* data)
{
    uint16_t write_pos = dma_get_write_pos();

    if (read_pos != write_pos)
    {
        dma_read_buffer(data, read_pos);
        return true;
    }

    return false;
}

/**
 * USART2 DMA mapping stm32f407 rm. Table 32. DMA1 request mapping
 * 
 * RX -> DMA1 Stream5 Channel4
 * TX -> DMA1 Stream6 Channel4
 */
void dma_setup(void)
{
    rcc_periph_clock_enable(RCC_DMA1);

    dma_stream_reset(USING_DMA, USING_STREAM);

    dma_channel_select(USING_DMA, USING_STREAM, USING_CHANNEL);

    dma_set_peripheral_address(USING_DMA, USING_STREAM, (uint32_t)&USART_DR(USART2));

    dma_set_memory_address(USING_DMA, USING_STREAM, (uint32_t)buffer);

    dma_set_transfer_mode(USING_DMA, USING_STREAM, DMA_SxCR_DIR_PERIPHERAL_TO_MEM);

    dma_set_number_of_data(USING_DMA, USING_STREAM, BUFFER_LENGTH);

    dma_set_peripheral_size(USING_DMA, USING_STREAM, DMA_SxCR_PSIZE_8BIT);
    dma_set_memory_size(USING_DMA, USING_STREAM, DMA_SxCR_MSIZE_8BIT);
    dma_disable_peripheral_increment_mode(USING_DMA, USING_STREAM);
    dma_enable_memory_increment_mode(USING_DMA, USING_STREAM);
    dma_enable_circular_mode(USING_DMA, USING_STREAM);

    dma_enable_stream(USING_DMA, USING_STREAM);
}

void dma_teardown(void)
{
    dma_disable_stream(USING_DMA, USING_STREAM);

    dma_disable_memory_increment_mode(USING_DMA, USING_STREAM);

    dma_stream_reset(USING_DMA, USING_STREAM);

    rcc_periph_clock_disable(RCC_DMA1);
}
