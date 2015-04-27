/*
  M24M01 - 1Mb I2C EEPROM

  NOTE: current code does not cater for identification page

  William Tan, Mar 2015
*/

#ifndef _M24M01_H
#define _M24M01_H

#include "stm32f4xx_hal.h"

#define EEPROM_EnableWriteControl()     HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET); //set WC# to high
#define EEPROM_DisableWriteControl()    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET); //set WC# to low

/*
Device Address (accessing memory)
===============================================================================
  MSB                           LSB
Bits 7 6 5 4 |   3  2   |  1   |  0       
     1 0 1 0    A2 A1     A16    R/W#
    Slave ID    Device    Page
                Select   Select
===============================================================================
Device Address (accessing identification page)
===============================================================================
  MSB                           LSB
Bits 7 6 5 4 |   3  2   |  1   |  0       
     1 0 1 1    A2 A1      X    R/W#
    Slave ID    Device    Don't
                Select    Care
===============================================================================
*/
#define EEPROM_SLAVE_DEV0_ADDR    0xA0
#define EEPROM_SLAVE_DEV1_ADDR    0xA4
#define EEPROM_SLAVE_DEV2_ADDR    0xA8
#define EEPROM_SLAVE_DEV3_ADDR    0xAC
#define EEPROM_ID_PG_DEV0_ADDR    0xB0
#define EEPROM_ID_PG_DEV1_ADDR    0xB4
#define EEPROM_ID_PG_DEV2_ADDR    0xB8
#define EEPROM_ID_PG_DEV3_ADDR    0xBC

#define EEPROM_PAGE_SELECT_H      0x02 //10000h to 1FFFFh (17-bit memory address) - P/S bit set to High (MSB)
#define EEPROM_PAGE_SELECT_L      0x00 //00000h to 0FFFFh (17-bit memory address) - P/s bit set to Low (MSB)
#define EEPROM_WRITE_TO_SLAVE     0x00
#define EEPROM_READ_FROM_SLAVE    0x01

uint32_t EEPROM_MultiByte_Write(I2C_HandleTypeDef *hi2c, uint32_t mem_addr, uint8_t *pData, uint16_t size);
uint32_t EEPROM_MultiByte_Selective_Read(I2C_HandleTypeDef *hi2c, uint32_t mem_addr, uint8_t *pData, uint16_t size);

static HAL_StatusTypeDef I2C_MasterRequestWrite(I2C_HandleTypeDef *hi2c, uint16_t DevAddress, uint32_t Timeout);
static HAL_StatusTypeDef I2C_WaitOnFlagUntilTimeout(I2C_HandleTypeDef *hi2c, uint32_t Flag, FlagStatus Status, uint32_t Timeout);
static HAL_StatusTypeDef I2C_WaitOnMasterAddressFlagUntilTimeout(I2C_HandleTypeDef *hi2c, uint32_t Flag, uint32_t Timeout);
static HAL_StatusTypeDef I2C1_MasterRequestRead(I2C_HandleTypeDef *hi2c, uint16_t DevAddress, uint32_t Timeout);

#endif
