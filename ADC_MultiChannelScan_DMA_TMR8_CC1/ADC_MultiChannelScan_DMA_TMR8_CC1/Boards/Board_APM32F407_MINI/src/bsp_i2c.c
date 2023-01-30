/*!
 * @file        bsp_i2c.c
 *
 * @brief       I2C board support package body
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
#include "bsp_i2c.h"
#include "bsp_delay.h"

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
 * @brief       LCD touch I2C GPIO Init
 *
 * @param       None
 *
 * @retval      None
 *
 */
void I2C_GPIOInit(void)
{
    GPIO_Config_T     gpioConfigStruct;
    
    RCM_EnableAHB1PeriphClock(LCD_I2C_SCL_GPIO_CLK);
    RCM_EnableAHB1PeriphClock(LCD_I2C_SDA_GPIO_CLK);
    
    gpioConfigStruct.pin = LCD_I2C_SCL_GPIO_PIN;
    gpioConfigStruct.mode = GPIO_MODE_OUT;
    gpioConfigStruct.speed = GPIO_SPEED_50MHz;
    gpioConfigStruct.otype = GPIO_OTYPE_OD;
    gpioConfigStruct.pupd = GPIO_PUPD_NOPULL;
    GPIO_Config(LCD_I2C_SCL_GPIO_PORT, &gpioConfigStruct);
    
    gpioConfigStruct.pin = LCD_I2C_SDA_GPIO_PIN;
    GPIO_Config(LCD_I2C_SDA_GPIO_PORT, &gpioConfigStruct);
    
    LCD_I2C_SCL_1;
    LCD_I2C_SDA_1;
}

static void I2C_Delay(void)
{
    uint8_t i;

    for (i = 0; i < 10*2; i++)
    {
    }
}

/*!
 * @brief       I2C start signal
 *
 * @param       None
 *
 * @retval      None
 *
 */
void I2C_Start(void)
{
    LCD_I2C_SDA_1;
    LCD_I2C_SCL_1;
    I2C_Delay();
    LCD_I2C_SDA_0;
    I2C_Delay();
    LCD_I2C_SCL_0;
    I2C_Delay();
}

/*!
 * @brief       I2C stop signal
 *
 * @param       None
 *
 * @retval      None
 *
 */
void I2C_Stop(void)
{
    LCD_I2C_SDA_0;
    LCD_I2C_SCL_1;
    I2C_Delay();
    LCD_I2C_SDA_1;
}

/*!
 * @brief       I2C wait ack
 *
 * @param       None
 *
 * @retval      Device Status
 *              @arg 1: Device ack error
 *              @arg 0: Device ack right
 *
 */
uint8_t I2C_WaitAck(void)
{
    uint8_t result;
    
    LCD_I2C_SDA_1;
    I2C_Delay();
    LCD_I2C_SCL_1;
    I2C_Delay();
    if(LCD_I2C_SDA_INPUT)
    {
        result = 1;
    }
    else
    {
        result = 0;
    }
    LCD_I2C_SCL_0;
    I2C_Delay();
    
    return result;
}

/*!
 * @brief       I2C ack signal
 *
 * @param       None
 *
 * @retval      None
 *
 */
void I2C_Ack(void)
{
    LCD_I2C_SDA_0;
    I2C_Delay();
    LCD_I2C_SCL_1;
    I2C_Delay();
    LCD_I2C_SCL_0;
    I2C_Delay();
    LCD_I2C_SDA_1;
}

/*!
 * @brief       I2C no ack signal
 *
 * @param       None
 *
 * @retval      None
 *
 */
void I2C_NoAck(void)
{
    LCD_I2C_SDA_1;
    I2C_Delay();
    LCD_I2C_SCL_1;
    I2C_Delay();
    LCD_I2C_SCL_0;
    I2C_Delay();
}

/*!
 * @brief       I2C send byte
 *
 * @param       data
 *
 * @retval      None
 *
 */
void I2C_SendByte(uint8_t data)
{
    uint8_t i;
    
    for(i = 0; i < 8; i++)
    {
        if(data & 0x80)
        {
            LCD_I2C_SDA_1;
        }
        else
        {
            LCD_I2C_SDA_0;
        }
        
        I2C_Delay();
        LCD_I2C_SCL_1;
        I2C_Delay();
        LCD_I2C_SCL_0;
        
        
        if(i == 7)
        {
            LCD_I2C_SDA_1;
        }
        data <<= 1;
        I2C_Delay();
    }
}

/*!
 * @brief       I2C read byte
 *
 * @param       None
 *
 * @retval      Data readed from device
 *
 */
uint8_t I2C_ReadByte(void)
{
    uint8_t i;
    uint8_t revData;
    
    revData = 0;
    
    for(i = 0; i < 8; i++)
    {
        revData <<= 1;

        LCD_I2C_SCL_1;
        I2C_Delay();
        if(LCD_I2C_SDA_INPUT)
        {
            revData++;
        }
        LCD_I2C_SCL_0;
        I2C_Delay();
    }
    
    return revData;
}

/*!
 * @brief       I2C read data
 *
 * @param       deviceAddr
 *
 * @param       buffer
 *
 * @param       readNumber
 *
 * @retval      Status code
 *
 */
uint8_t I2C_ReadData(uint8_t deviceAddr, uint8_t* buffer, uint16_t readNumber)
{
    I2C_Start();
    
    I2C_SendByte(deviceAddr | LCD_I2C_RD_BIT);
    
    if(I2C_WaitAck())
    {
        goto deviceError;
    }
    
    while(readNumber)
    {
        if(readNumber == 1)
        {
            I2C_NoAck();
            
            I2C_Stop();
        }
        
        *buffer = I2C_ReadByte();
        
        buffer++;
        
        readNumber--;
        
        I2C_Ack();
    }
    
    I2C_Stop();
    return 0;

    deviceError:
    I2C_Stop();
    return 1;
}

/*!
 * @brief       I2C write data
 *
 * @param       deviceAddr
 *
 * @param       buffer
 *
 * @param       writeNumber
 *
 * @retval      Status code
 *
 */
uint8_t I2C_WriteData(uint8_t deviceAddr, uint8_t* buffer, uint16_t writeNumber)
{
    uint16_t timeout;
    
    I2C_Stop();
    
    for( timeout = 0; timeout < 1000; timeout++)
    {
        I2C_Start();
        
        I2C_SendByte(deviceAddr | LCD_I2C_WR_BIT);
        
        if(I2C_WaitAck() == 0)
        {
            break;
        }
    }
    
    if(timeout == 1000)
    {
        goto deviceError;
    }
    
    while(writeNumber--)
    {
        I2C_SendByte(*buffer);
        
        if(I2C_WaitAck())
        {
            goto deviceError;
        }
        
        buffer++;
    }
    
    I2C_Stop();
    return 0;

    deviceError:
    I2C_Stop();
    return 1;
}

/**@} end of group APM32F407_MINI_Functions */
/**@} end of group Board_APM32F407_MINI */
/**@} end of group Board */
