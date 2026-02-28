#ifndef DMA_H
#define DMA_H

#include "common-defines.h"

void dma_setup(void);
bool dma_read(uint8_t* data);
void dma_teardown(void);
bool dma_data_available(void);

#endif