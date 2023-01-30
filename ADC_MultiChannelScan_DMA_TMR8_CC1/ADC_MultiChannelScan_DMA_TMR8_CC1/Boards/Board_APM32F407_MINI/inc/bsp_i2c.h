/*!
 * @file        bsp_i2c.h
 *
 * @brief       Header for bsp_i2c.c module
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
#ifndef _BSP_I2C_H
#define _BSP_I2C_H

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

#define LCD_I2C_SCL_GPIO_PIN            GPIO_PIN_7
#define LCD_I2C_SCL_GPIO_PORT           GPIOD
#define LCD_I2C_SCL_GPIO_CLK            RCM_AHB1_PERIPH_GPIOD
#define LCD_I2C_SCL_SOURCE              GPIO_PIN_SOURCE_7

#define LCD_I2C_SDA_GPIO_PIN            GPIO_PIN_3
#define LCD_I2C_SDA_GPIO_PORT           GPIOD
#define LCD_I2C_SDA_GPIO_CLK            RCM_AHB1_PERIPH_GPIOD
#define LCD_I2C_SDA_SOURCE              GPIO_PIN_SOURCE_3

#define LCD_I2C_SDA_IN()               { LCD_I2C_SDA_GPIO_PORT->MODE &= ~(3 << (3 * 2)); LCD_I2C_SDA_GPIO_PORT->MODE |= 0 << 3 * 2; }
#define LCD_I2C_SDA_OUT()              { LCD_I2C_SDA_GPIO_PORT->MODE &= ~(3 << (3 * 2)); LCD_I2C_SDA_GPIO_PORT->MODE |= 1 << 3 * 2; }

#define LCD_I2C_SCL_1                   GPIO_SetBit(LCD_I2C_SCL_GPIO_PORT,LCD_I2C_SCL_GPIO_PIN)
#define LCD_I2C_SCL_0                   GPIO_ResetBit(LCD_I2C_SCL_GPIO_PORT,LCD_I2C_SCL_GPIO_PIN)

#define LCD_I2C_SDA_1                   GPIO_SetBit(LCD_I2C_SDA_GPIO_PORT,LCD_I2C_SDA_GPIO_PIN)
#define LCD_I2C_SDA_0                   GPIO_ResetBit(LCD_I2C_SDA_GPIO_PORT,LCD_I2C_SDA_GPIO_PIN)

#define LCD_I2C_SDA_INPUT               GPIO_ReadInputBit(LCD_I2C_SDA_GPIO_PORT,LCD_I2C_SDA_GPIO_PIN)

#define LCD_I2C_WR_BIT                  0
#define LCD_I2C_RD_BIT                  1

/**
  * @}
  */

/** @defgroup APM32F407_MINI_Functions
  @{
  */


/** function declaration*/
void I2C_GPIOInit(void);
uint8_t I2C_ReadData(uint8_t deviceAddr, uint8_t* buffer, uint16_t readNumber);
uint8_t I2C_WriteData(uint8_t deviceAddr, uint8_t* buffer, uint16_t writeNumber);

#endif

/**@} end of group APM32F407_MINI_Functions */
/**@} end of group Board_APM32F407_MINI */
/**@} end of group Board */
