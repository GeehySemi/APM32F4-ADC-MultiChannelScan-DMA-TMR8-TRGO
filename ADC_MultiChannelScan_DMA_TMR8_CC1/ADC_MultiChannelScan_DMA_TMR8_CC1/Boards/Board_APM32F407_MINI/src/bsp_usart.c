/*!
 * @file        bsp_usart.c
 *
 * @brief       Usart board support package body
 *
 * @version     V1.0.0
 *
 * @date        2022-05-25
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
#include "bsp_usart.h"
#include <stdio.h>

/** @addtogroup Board
  @{
*/

/** @addtogroup Board_APM32F407_MINI
  @{
*/

/** @defgroup APM32F407_MINI_Fuctions
  @{
  */

/*!
 * @brief       Redirect the fputc function
 *
 * @param       ch
 *
 * @param       f
 *
 * @retval      None
 *
 */
int fputc(int ch, FILE *f)
{
    /** send byte */
    USART_TxData(USART1, (uint8_t) ch);

    while (USART_ReadStatusFlag(DEBUG_USART, USART_FLAG_TXBE) == RESET)
    {
        
    }

    return (ch);
}

/*!
 * @brief       USART1 Initialization
 *
 * @param       None
 *
 * @retval      None
 *
 */
void USART1_Init(uint32_t baud)
{
    USART_Config_T usartConfigStruct;

    /** USART1 configuration */
    usartConfigStruct.baudRate = baud;
    usartConfigStruct.hardwareFlow = USART_HARDWARE_FLOW_NONE;
    usartConfigStruct.mode = USART_MODE_TX_RX;
    usartConfigStruct.parity = USART_PARITY_NONE;
    usartConfigStruct.stopBits = USART_STOP_BIT_1;
    usartConfigStruct.wordLength = USART_WORD_LEN_8B;
    APM_MINI_COMInit(COM1, &usartConfigStruct);
}

/*!
 * @brief       USART2 Initialization
 *
 * @param       None
 *
 * @retval      None
 *
 */
void USART2_Init(uint32_t baud)
{
    USART_Config_T usartConfigStruct;

    /** USART1 configuration */
    usartConfigStruct.baudRate = baud;
    usartConfigStruct.hardwareFlow = USART_HARDWARE_FLOW_NONE;
    usartConfigStruct.mode = USART_MODE_TX_RX;
    usartConfigStruct.parity = USART_PARITY_NONE;
    usartConfigStruct.stopBits = USART_STOP_BIT_1;
    usartConfigStruct.wordLength = USART_WORD_LEN_8B;
    APM_MINI_COMInit(COM2, &usartConfigStruct);
}

/*!
 * @brief       Serial port tramsimt data
 *
 * @param       pointer to date that need to be sent
 *
 * @retval      None
 *
 */
void USART_Write(USART_T* usart,uint8_t *dat, uint32_t count)
{
    while(count--)
    {
        while(USART_ReadStatusFlag(usart, USART_FLAG_TXBE) == RESET);
        USART_TxData(usart, *dat++);
    }
}

/**@} end of group APM32F407_MINI_Functions */
/**@} end of group Board_APM32F407_MINI */
/**@} end of group Board */
