/*
 * maincpp.h
 *
 *  Created on: Sep 30, 2024
 *      Author: User
 */

#ifndef SRC_MAINCPP_H_
#define SRC_MAINCPP_H_

#include "stm32g4xx_hal.h"
#include "stm32g4xx_nucleo.h"
#include <stdio.h>

//#define DAC_TEST
#define NOISE_GENERATOR

static const uint_fast8_t NFLT = 20;
static const uint_fast8_t STP_LN = 4;
static const uint_fast8_t LEN_DATA = 28;

#define decinst_t adecoder_t<NFLT,STP_LN,LEN_DATA>

#define INLN __attribute__((__always_inline__)) inline

#define DMA_SEI {__HAL_DMA_ENABLE_IT(&hdma_memtomem_dma1_channel2,DMA_IT_TC);__HAL_DMA_ENABLE_IT(&hdma_memtomem_dma2_channel2,DMA_IT_TC);}
#define DMA_CLI {__HAL_DMA_DISABLE_IT(&hdma_memtomem_dma1_channel2,DMA_IT_TC);__HAL_DMA_DISABLE_IT(&hdma_memtomem_dma2_channel2,DMA_IT_TC);}
#define DMA_CRITICAL(action) {DMA_CLI {action;} DMA_SEI}

//#define OPAMP_SET_PGA(hopamp,gain) MODIFY_REG(hopamp->CSR,OPAMP_CSR_PGGAIN,gain)
void HAL_FMAC_OutputDataReadyCallback(FMAC_HandleTypeDef *hfmac) __attribute__((section (".ccmram")));
void DMA_DoubleBuffCallback(DMA_HandleTypeDef* d) __attribute__((section (".ccmram")));
void DecoderHandler(void) __attribute__((section (".ccmram")));
uint32_t OpampPGA(int_fast8_t Inc);

#endif /* SRC_MAINCPP_H_ */
