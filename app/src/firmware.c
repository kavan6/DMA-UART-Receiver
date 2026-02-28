#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/memorymap.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/dma.h>
#include <libopencm3/stm32/usart.h>
#include <libopencm3/cm3/scb.h>

#include "core/uart.h"
#include "core/system.h"

#define BOOTLOADER_SIZE     (0x8000U)

#define LED_PORT            (GPIOA)
#define LED_PIN_D2          (GPIO6)
#define LED_PIN_D3          (GPIO7)

#define USART2_PORT         (GPIOA)
#define USART2_CTS_PIN      (GPIO0)
#define USART2_RTS_PIN      (GPIO1)
#define USART2_TX_PIN       (GPIO2)
#define USART2_RX_PIN       (GPIO3)
#define USART2_CK_PIN       (GPIO4)

#define BUF_LEN 64

uint8_t buffer[BUF_LEN];

static uint16_t read_pos = 0;

static void vector_setup(void)
{
    SCB_VTOR = BOOTLOADER_SIZE;
}

static void gpio_setup(void)
{
    rcc_periph_clock_enable(RCC_GPIOA);

    gpio_mode_setup(USART2_PORT, GPIO_MODE_AF, GPIO_PUPD_NONE, (USART2_TX_PIN | USART2_RX_PIN));

    gpio_set_af(USART2_PORT, GPIO_AF7, (USART2_TX_PIN | USART2_RX_PIN));
}

/**
 * USART2 DMA mapping stm32f407 rm. Table 32. DMA1 request mapping
 * 
 * RX -> DMA1 Stream5 Channel4
 * TX -> DMA1 Stream6 Channel4
 */
static void dma_setup(void)
{
    rcc_periph_clock_enable(RCC_DMA1);

    dma_stream_reset(DMA1, DMA_STREAM5);

    dma_channel_select(DMA1, DMA_STREAM5, DMA_SxCR_CHSEL_4);

    dma_set_peripheral_address(DMA1, DMA_STREAM5, (uint32_t)&USART_DR(USART2));

    dma_set_memory_address(DMA1, DMA_STREAM5, (uint32_t)buffer);

    dma_set_transfer_mode(DMA1, DMA_STREAM5, DMA_SxCR_DIR_PERIPHERAL_TO_MEM);

    dma_set_number_of_data(DMA1, DMA_STREAM5, BUF_LEN);

    dma_set_peripheral_size(DMA1, DMA_STREAM5, DMA_SxCR_PSIZE_8BIT);
    dma_set_memory_size(DMA1, DMA_STREAM5, DMA_SxCR_MSIZE_8BIT);

    dma_disable_peripheral_increment_mode(DMA1, DMA_STREAM5);
    dma_enable_memory_increment_mode(DMA1, DMA_STREAM5);
    dma_enable_circular_mode(DMA1, DMA_STREAM5);

    dma_enable_stream(DMA1, DMA_STREAM5);
}

static uint16_t dma_get_write_pos(void)
{
    return BUF_LEN - dma_get_number_of_data(DMA1, DMA_STREAM5);
}

static void uart_process_dma(void)
{
    uint16_t write_pos = dma_get_write_pos();

    while (read_pos != write_pos)
    {
        uint8_t byte = buffer[read_pos];

        usart_send_blocking(USART2, byte + 1);

        read_pos++;
        if (read_pos >= BUF_LEN)
        {
            read_pos = 0;
        }
    }
}

int main(void)
{
    vector_setup();
    system_setup();

    gpio_setup();

    uart_setup();
    dma_setup();

    usart_enable_rx_dma(USART2);

    while (1)
    {
        uart_process_dma();
    }

    uart_teardown();

    // Never return
    return 0;
}