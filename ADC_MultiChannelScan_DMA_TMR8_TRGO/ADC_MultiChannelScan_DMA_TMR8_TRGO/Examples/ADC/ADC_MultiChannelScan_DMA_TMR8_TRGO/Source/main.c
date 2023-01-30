/*!
 * @file        main.c
 *
 * @brief       Main program body
 *
 * @version     V1.0.2
 *
 * @date        2022-06-23
 *
 * @attention
 *
 *  Copyright (C) 2021-2022 Geehy Semiconductor
 *
 *  You may not use this file except in compliance with the
 *  GEEHY COPYRIGHT NOTICE (GEEHY SOFTWARE PACKAGE LICENSE).
 *
 *  The program is only for reference, which is distributed in the hope
 *  that it will be usefull and instructional for customers to develop
 *  their software. Unless required by applicable law or agreed to in
 *  writing, the program is distributed on an "AS IS" BASIS, WITHOUT
 *  ANY WARRANTY OR CONDITIONS OF ANY KIND, either express or implied.
 *  See the GEEHY SOFTWARE PACKAGE LICENSE for the governing permissions
 *  and limitations under the License.
 */

/* Includes */
#include "main.h"
#include "Board.h"
#include "stdio.h"
#include "apm32f4xx_gpio.h"
#include "apm32f4xx_adc.h"
#include "apm32f4xx_misc.h"
#include "apm32f4xx_usart.h"
#include "apm32f4xx_tmr.h"
/** @addtogroup Examples
  @{
  */

/** @addtogroup ADC_MultiChannelScan
  @{
  */

/** @defgroup ADC_MultiChannelScan_Macros Macros
  @{
*/

/* printf function configs to USART1*/
#define DEBUG_USART  USART1

/**@} end of group ADC_MultiChannelScan_Macros*/

/** @defgroup ADC_MultiChannelScan_Functions Functions
  @{
  */

/* Delay */
void Delay(uint32_t count);
/* ADC init */
void ADC_Init(void);

void ADC_MultiChannelPolling(void);

/* save adc data*/
#define ADC_CH_SIZE         3
#define ADC_DR_ADDR         ((uint32_t)ADC1_BASE + 0x4C)

uint16_t adcData[ADC_CH_SIZE];

 void TMR8_TRGO_Init(void)
 {
    TMR_BaseConfig_T TMR_TimeBaseStruct;

    RCM_EnableAPB2PeriphClock(RCM_APB2_PERIPH_TMR8);
	 
    TMR_TimeBaseStruct.clockDivision = TMR_CLOCK_DIV_1;
    TMR_TimeBaseStruct.countMode = TMR_COUNTER_MODE_UP;
    TMR_TimeBaseStruct.division = 16800;
    TMR_TimeBaseStruct.period = 2000-1;
    TMR_ConfigTimeBase(TMR8, &TMR_TimeBaseStruct);
	 
    TMR_SelectOutputTrigger(TMR8,TMR_TRGO_SOURCE_UPDATE);
	 	 
    TMR_EnableAutoReload(TMR8);
    TMR_Enable(TMR8);   
 } 


/*!
 * @brief     Main program
 *
 * @param     None
 *
 * @retval    None
 */
int main(void)
{
    USART_Config_T usartConfigStruct;

    /* Configures LED2 and LED3 */
    APM_MINI_LEDInit(LED2);
    APM_MINI_LEDInit(LED3);

    /* USART configuration */
    USART_ConfigStructInit(&usartConfigStruct);
    usartConfigStruct.baudRate = 115200;
    usartConfigStruct.mode = USART_MODE_TX_RX;
    usartConfigStruct.parity = USART_PARITY_NONE;
    usartConfigStruct.stopBits = USART_STOP_BIT_1;
    usartConfigStruct.wordLength = USART_WORD_LEN_8B;
    usartConfigStruct.hardwareFlow = USART_HARDWARE_FLOW_NONE;

    /* COM1 init*/
    APM_MINI_COMInit(COM1, &usartConfigStruct);
	
	  TMR8_TRGO_Init();
  
    /* ADC1 initialization */
    ADC_Init();

    while (1)
    {

    }
}

/*!
 * @brief     DMA Init
 *
 * @param     None
 *
 * @retval    None
 */
void DMA_Init(void)
{
    DMA_Config_T dmaConfig;

    RCM_EnableAHB1PeriphClock(RCM_AHB1_PERIPH_DMA2);

    dmaConfig.peripheralBaseAddr = ADC_DR_ADDR;
    dmaConfig.memoryBaseAddr = (uint32_t)&adcData;
    dmaConfig.dir = DMA_DIR_PERIPHERALTOMEMORY;
    dmaConfig.bufferSize = ADC_CH_SIZE;
    dmaConfig.peripheralInc = DMA_PERIPHERAL_INC_DISABLE;
    dmaConfig.memoryInc = DMA_MEMORY_INC_ENABLE;
    dmaConfig.peripheralDataSize = DMA_PERIPHERAL_DATA_SIZE_HALFWORD;
    dmaConfig.memoryDataSize = DMA_MEMORY_DATA_SIZE_HALFWORD;
    dmaConfig.loopMode = DMA_MODE_CIRCULAR;

    dmaConfig.priority = DMA_PRIORITY_HIGH;
    dmaConfig.fifoMode = DMA_FIFOMODE_DISABLE;
    dmaConfig.fifoThreshold = DMA_FIFOTHRESHOLD_HALFFULL;
    dmaConfig.memoryBurst = DMA_MEMORYBURST_SINGLE;
    dmaConfig.peripheralBurst = DMA_PERIPHERALBURST_SINGLE;
    dmaConfig.channel = DMA_CHANNEL_0;
    DMA_Config(DMA2_Stream0,&dmaConfig);
    
    /* Clear DMA TF flag*/
    DMA_ClearIntFlag(DMA2_Stream0, DMA_INT_TCIFLG2);
    /* Enable DMA Interrupt*/
    DMA_EnableInterrupt(DMA2_Stream0, DMA_INT_TCIFLG);
    DMA_EnableInterrupt(DMA2_Stream0, DMA_INT_TEIFLG);
    NVIC_EnableIRQRequest(DMA2_STR0_IRQn, 1, 0);

    DMA_Enable(DMA2_Stream0);
}

/*!
 * @brief     ADC Init
 *
 * @param     None
 *
 * @retval    None
 */
void ADC_Init(void)
{
    GPIO_Config_T           gpioConfig;
    ADC_Config_T            adcConfig;
    ADC_CommonConfig_T      adcCommonConfig;

    /* Enable GPIOA clock */
    RCM_EnableAHB1PeriphClock(RCM_AHB1_PERIPH_GPIOA);

    /* ADC channel 0 configuration */
    GPIO_ConfigStructInit(&gpioConfig);
    gpioConfig.mode    = GPIO_MODE_AN;
    gpioConfig.pupd    = GPIO_PUPD_NOPULL;
    gpioConfig.pin     = GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2;
    GPIO_Config(GPIOA, &gpioConfig);

    /* Enable ADC clock */
    RCM_EnableAPB2PeriphClock(RCM_APB2_PERIPH_ADC1);

    /* ADC configuration */
    ADC_Reset();

    adcCommonConfig.mode            = ADC_MODE_INDEPENDENT;
    adcCommonConfig.prescaler       = ADC_PRESCALER_DIV2;
    adcCommonConfig.accessMode      = ADC_ACCESS_MODE_DISABLED;
    adcCommonConfig.twoSampling     = ADC_TWO_SAMPLING_20CYCLES;
    ADC_CommonConfig(&adcCommonConfig);

    ADC_ConfigStructInit(&adcConfig);
    adcConfig.resolution            = ADC_RESOLUTION_12BIT;
    adcConfig.scanConvMode          = ENABLE;
    adcConfig.continuousConvMode    = DISABLE;
    adcConfig.dataAlign             = ADC_DATA_ALIGN_RIGHT;
    adcConfig.extTrigEdge           = ADC_EXT_TRIG_EDGE_RISING;
    adcConfig.extTrigConv           = ADC_EXT_TRIG_CONV_TMR8_TRGO;
    adcConfig.nbrOfChannel          = ADC_CH_SIZE;
    ADC_Config(ADC1, &adcConfig);

    /* ADC channel Convert configuration */
    ADC_ConfigRegularChannel(ADC1, ADC_CHANNEL_0, 1, ADC_SAMPLETIME_480CYCLES);
    ADC_ConfigRegularChannel(ADC1, ADC_CHANNEL_1, 2, ADC_SAMPLETIME_480CYCLES);
    ADC_ConfigRegularChannel(ADC1, ADC_CHANNEL_2, 3, ADC_SAMPLETIME_480CYCLES);

    /* Config DMA*/
    DMA_Init();

    /* Enable ADC DMA Request*/
    ADC_EnableDMARequest(ADC1);

    /* Enable ADC DMA*/
    ADC_EnableDMA(ADC1);

    /* Enable ADC */
    ADC_Enable(ADC1);

}

/*!
 * @brief     ADC multi channel polling
 *
 * @param     None
 *
 * @retval    None
 */
void ADC_MultiChannelPolling(void)
{
    float voltage;
    uint8_t index;

    for(index = 0; index < ADC_CH_SIZE; index++)
    {
        voltage = (adcData[index] * 3300.0 ) / 4095.0;

        printf("ADC CH[%d] voltage = %.3f mV\r\n", index, voltage);
    }
    printf("\r\n");
}

/*!
 * @brief     Delay
 *
 * @param     count:  delay count
 *
 * @retval    None
 */
void Delay(uint32_t count)
{
    volatile uint32_t delay = count;

    while (delay--);
}

/*!
 * @brief     Redirect C Library function printf to serial port.
 *            After Redirection, you can use printf function.
 *
 * @param     ch:  The characters that need to be send.
 *
 * @param     *f:  pointer to a FILE that can recording all information
 *            needed to control a stream
 *
 * @retval    The characters that need to be send.
 */
int fputc(int ch, FILE* f)
{
    /* send a byte of data to the serial port */
    USART_TxData(DEBUG_USART, (uint8_t)ch);

    /* wait for the data to be send  */
    while (USART_ReadStatusFlag(DEBUG_USART, USART_FLAG_TXBE) == RESET);

    return (ch);
}

/**@} end of group ADC_MultiChannelScan_Functions */
/**@} end of group ADC_MultiChannelScan */
/**@} end of group Examples */
void DMA2_STR0_IRQHandler(void)
{
    if(DMA_ReadIntFlag(DMA2_Stream0, DMA_INT_TCIFLG0) == SET)
    {
	     DMA_ClearIntFlag(DMA2_Stream0, DMA_INT_TCIFLG0);
	     ADC_MultiChannelPolling();
    }
    else if(DMA_ReadStatusFlag(DMA2_Stream0, DMA_FLAG_TEIFLG0) == SET)
    {
       DMA_ClearStatusFlag(DMA2_Stream0, DMA_FLAG_TEIFLG0);
    }
}