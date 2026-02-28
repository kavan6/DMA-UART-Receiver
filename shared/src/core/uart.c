#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/usart.h>
#include <libopencm3/cm3/nvic.h>

#include "core/ring-buffer.h"
#include "core/uart.h"
#include "core/dma.h"

#define DATA_BITS        (8)
#define PARITY_BITS      (0)
#define STOP_BITS        (1)
#define BAUD_RATE        (115200)

#define RX_BUFFER_SIZE   (128)
#define TX_BUFFER_SIZE   (64)

static uint8_t rx_buffer[RX_BUFFER_SIZE];
static uint8_t tx_buffer[TX_BUFFER_SIZE];

static ring_buffer_t rx_rb;
static ring_buffer_t tx_rb;

static void uart_rx_process(void)
{
    uint8_t byte;

    while(dma_read(&byte))
    {
        ring_buffer_write(&rx_rb, byte);
    }
}

uint32_t uart_read(uint8_t* data, const uint32_t length)
{
    uart_rx_process();

    uint32_t bytes_read = 0;

    for (; bytes_read < length; bytes_read++)
    {
        if (!ring_buffer_read(&rx_rb, &data[bytes_read]))
        {
            return bytes_read;
        }
    }

    return bytes_read;
}

void uart_setup(void)
{
    rcc_periph_clock_enable(RCC_USART2);

    usart_set_mode(USART2,  USART_MODE_TX_RX);
    usart_set_flow_control(USART2, USART_FLOWCONTROL_NONE);
    usart_set_databits(USART2, DATA_BITS);
    usart_set_baudrate(USART2, BAUD_RATE);
    usart_set_parity(USART2, PARITY_BITS);
    usart_set_stopbits(USART2, STOP_BITS);

    ring_buffer_setup(&rx_rb, rx_buffer, RX_BUFFER_SIZE);
    ring_buffer_setup(&tx_rb, tx_buffer, TX_BUFFER_SIZE);

    dma_setup();

    (void)USART_SR(USART2);
    (void)USART_DR(USART2);

    usart_enable_rx_dma(USART2);
    
    usart_enable(USART2);
}

void uart_teardown(void)
{
    usart_disable_rx_dma(USART2);

    usart_disable(USART2);

    dma_teardown();

    rcc_periph_clock_disable(RCC_USART2);
}


void uart_write(uint8_t* data, const uint32_t length)
{
    for (uint32_t i = 0; i < length; i++)
    {
        uart_write_byte(data[i]);
    }
}

void uart_write_byte(uint8_t data)
{
    usart_send_blocking(USART2, (uint16_t)data);
}


uint8_t uart_read_byte(void)
{
    uint8_t byte = 0;
    (void)uart_read(&byte, 1U);
    return byte;
}

bool uart_data_available(void)
{
    uart_rx_process();
    return !ring_buffer_empty(&rx_rb);
}
