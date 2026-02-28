#ifndef DMA_H
#define DMA_H

#include "common-defines.h"

void dma_setup(void);
void dma_read_buffer(uint8_t* data, uint16_t pos);
bool dma_read(uint8_t* data);
bool dma_data_available(void);

#endif