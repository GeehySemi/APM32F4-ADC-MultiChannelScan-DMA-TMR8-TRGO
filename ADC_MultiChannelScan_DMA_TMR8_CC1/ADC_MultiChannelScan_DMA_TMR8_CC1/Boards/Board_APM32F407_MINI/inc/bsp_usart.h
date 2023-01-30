/*!
 * @file        bsp_usart.h
 *
 * @brief       Header for bsp_usart.c module
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

/* Define to prevent recursive inclusion */
#ifndef _BSP_USART_H
#define _BSP_USART_H

/* Includes */
#include "main.h"

/** @addtogroup Board
  @{
*/

/** @addtogroup Board_APM32F407_MINI
  @{
*/

/** @defgroup APM32F407_MINI_Macros
  @{
  */

#define DEBUG_USART USART1

/**
  * @}
  */

/** @defgroup APM32F407_MINI_Functions
  @{
  */

/** function declaration*/
void USART1_Init(uint32_t baud);
void USART2_Init(uint32_t baud);
void USART_Write(USART_T* usart,uint8_t *dat, uint32_t count);
#endif

/**@} end of group APM32F407_MINI_Functions */
/**@} end of group Board_APM32F407_MINI */
/**@} end of group Board */
