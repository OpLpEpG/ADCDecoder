/*
 * maincpp.cpp
 *
 *  Created on: Sep 30, 2024
 *      Author: User
 * 
 */

#include "maincpp.h"
#include "main.h"
#include "tim.h"
#include "opamp.h"
#include "adc.h"
#include "fmac.h"
#include "dac.h"
#include "dma.h"
#include "stm32g4xx_hal_fmac.h"

#include "adecoder.h"
#include "packer.h"
#include "ndm.h"
#include "GenExample.h"

extern __IO uint32_t BspButtonState;

//extern void inner_Encode(const void* inData, uint8_t* outData, const GeneratedItem_t* metadata, uint8_t lenmetadata);

static const uint_fast8_t NFLT = 20;
static const uint_fast8_t STP_LN = 4;
static const uint_fast8_t LEN_DATA = 28;

typedef adecoder_t<NFLT, STP_LN, LEN_DATA> decinst_t;

static decinst_t decoder;
static ndm_t ndmData;

static int16_t dmaBuf[decoder.dmalen];

void DMA_DoubleBuffCallback(DMA_HandleTypeDef* d) __attribute__((section (".ccmram")));
void DecoderRun(void) __attribute__((section (".ccmram")));

void HAL_FMAC_OutputDataReadyCallback(FMAC_HandleTypeDef *hfmac)
{
	/* Prevent unused argument(s) compilation warning */
	UNUSED(hfmac);

	HAL_DMA_Start_IT(&hdma_memtomem_dma1_channel2,
			(uint32_t)dmaBuf,
			decoder.NextDma(), decoder.dmalen/2);
	 if (decoder.dma_status == 0)
	 	HAL_DMA_Start_IT(&hdma_memtomem_dma2_channel2,
	 		(uint32_t)dmaBuf,
	 		decoder.NextDmaEx(), decoder.dmalen/2);
}

void DMA_DoubleBuffCallback(DMA_HandleTypeDef* d)
{	
	if (++decoder.dma_status >= 2)
	{
		decoder.NextDataIRQ();
	}
}

 void DecoderRun(void)
 {
 	decoder.NextDataPRG();
 }

extern "C" void main_cpp_begin(void)
{
	if (HAL_TIM_OC_Start(&htim4, TIM_CHANNEL_4) != HAL_OK)
	{
		/* Starting Error */
		Error_Handler();
	}
	if (HAL_TIM_Base_Start(&htim1) != HAL_OK)
	{
		/* Starting Error */
		Error_Handler();
	}
	if (HAL_TIM_Base_Start(&htim4) != HAL_OK)
	{
		/* Starting Error */
		Error_Handler();
	}
	if (HAL_OPAMP_Start(&hopamp3) != HAL_OK)
	{
		/* Starting Error */
		Error_Handler();
	}
	

	static const int16_t ni = (int16_t)(1.0 / NFLT * 0x8000);
	int16_t FIR_COEF[NFLT];
	for (uint_fast8_t i = 0; i < NFLT; i++)
		FIR_COEF[i] = ni;

	FMAC_FilterConfigTypeDef sFmacConfig;	  /* declare a filter configuration structure */
	sFmacConfig.CoeffBaseAddress = 0;		  /* Set the coefficient buffer base address */
	sFmacConfig.CoeffBufferSize = NFLT;		  /* Set the coefficient buffer size to the number of coeffs */
	sFmacConfig.InputBaseAddress = NFLT;	  /* Set the Input buffer base address to the next free address */
	sFmacConfig.InputBufferSize = NFLT;		  /* Set the input buffer size greater than the number of coeffs */
	sFmacConfig.InputThreshold = 0;			  /* Set the input watermark to zero since we are using DMA */
	sFmacConfig.OutputBaseAddress = NFLT * 2; /* Set the Output buffer base address to the next free address */
	sFmacConfig.OutputBufferSize = 1;		  /* Set the output buffer size */
	sFmacConfig.OutputThreshold = 0;		  /* Set the output watermark to zero since we are using DMA */
	sFmacConfig.pCoeffA = NULL;				  /* No A coefficients since FIR */
	sFmacConfig.CoeffASize = 0;
	sFmacConfig.pCoeffB = (int16_t *)FIR_COEF;				   /* Pointer to the coefficients in memory */
	sFmacConfig.CoeffBSize = NFLT;							   /* Number of coefficients */
	sFmacConfig.Filter = FMAC_FUNC_CONVO_FIR;				   /* Select FIR filter function */
	sFmacConfig.InputAccess = FMAC_BUFFER_ACCESS_DMA;		   /*Buffer handled by an external IP (ADC for instance)  */
	sFmacConfig.OutputAccess = FMAC_BUFFER_ACCESS_DMA;		   /* Enable DMA output transfer */
	sFmacConfig.Clip = FMAC_CLIP_ENABLED;					   /* Enable clipping of the output at 0x7FFF and 0x8000 */
	sFmacConfig.P = NFLT;									   /* P parameter contains number of coefficients */
	sFmacConfig.Q = 0;										   /* Q parameter is not used */
	sFmacConfig.R = 0;										   /* R parameter contains the post-shift value (none) */
	if (HAL_FMAC_FilterConfig(&hfmac, &sFmacConfig) != HAL_OK) /* Configure the FMAC */
		Error_Handler();									   /* Configuration Error */

	uint16_t len = 1;
#ifdef DAC_TEST
	HAL_DAC_Start(&hdac2, DAC_CHANNEL_1);

	if (HAL_FMAC_FilterStart(&hfmac, (int16_t *)&DAC2->DHR12R1, &len) != HAL_OK)
		Error_Handler();
#else
	if (HAL_DMA_RegisterCallback(&hdma_memtomem_dma2_channel2, HAL_DMA_XFER_CPLT_CB_ID, DMA_DoubleBuffCallback) != HAL_OK)
		Error_Handler();
	if (HAL_DMA_RegisterCallback(&hdma_memtomem_dma1_channel2, HAL_DMA_XFER_CPLT_CB_ID, DMA_DoubleBuffCallback) != HAL_OK)
		Error_Handler();
	len = decoder.dmalen;
	if (HAL_FMAC_FilterStart(&hfmac, dmaBuf, &len) != HAL_OK)
		Error_Handler();
#endif

	//	 HAL_ADC_Start_DMA(&hadc2,(uint32_t*) &DAC2->DHR12R1, 1); // test
	HAL_ADC_Start_DMA(&hadc2, (uint32_t *)&FMAC->WDATA, 1);

	while (1)
	{
		 DecoderRun();
		
		if (decoder.state < 0) BSP_LED_Off(LED_GREEN);
		else BSP_LED_On(LED_GREEN);

		if (decoder.CodeReadyFlag)
		{
			decoder.CodeReadyFlag = false;
			Decode(decoder.Code, &ndmData);
		}

		/* -- Sample board code for User push-button in interrupt mode ---- */
		if (BspButtonState == BUTTON_PRESSED)
		{
			/* Update button state */
			BspButtonState = BUTTON_RELEASED;
			/* -- Sample board code to toggle led ---- */
			BSP_LED_Toggle(LED_GREEN);

			/* ..... Perform your action ..... */
		}
	}
}
