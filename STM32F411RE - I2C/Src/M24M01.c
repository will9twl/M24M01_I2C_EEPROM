/*
  M24M01 - 1Mb I2C EEPROM

  NOTE: current code does not cater for identification page
  
  William Tan, Mar 2015
*/

#include "stm32f4xx_hal.h"
#include "M24M01.h"

uint16_t device_addr_write;
uint16_t device_addr_read;
uint16_t memory_addr;

uint32_t EEPROM_MultiByte_Write(I2C_HandleTypeDef *hi2c, uint32_t mem_addr, uint8_t *pData, uint16_t size)
{
  uint8_t device_addr_write, data_to_write[30];
  
  //configure slave address, page select bit and read/write# bit
  if(mem_addr >= 0x10000){ //memory address points to 10000h to 1FFFFh
    device_addr_write = EEPROM_SLAVE_DEV0_ADDR | EEPROM_PAGE_SELECT_H | EEPROM_WRITE_TO_SLAVE;  
  }else{ //memory address points to 00000h to 0FFFFh
    device_addr_write = EEPROM_SLAVE_DEV0_ADDR | EEPROM_PAGE_SELECT_L | EEPROM_WRITE_TO_SLAVE;  
  }
  
  uint16_t i;
  //configure memory address  
  data_to_write[0] = mem_addr>>8; //memory address MSB 
  data_to_write[1] = mem_addr&0x00FF; //memory address LSB 
  //load data to write
  for(i=2; i<size+2; i++){
    data_to_write[i] = pData[i-2];
  }
  
  //write to FRAM
  uint32_t status;
  status = HAL_I2C_Master_Transmit(hi2c, (uint8_t)device_addr_write, &data_to_write[0], size+2, 1);
  
  return status;
}

uint32_t EEPROM_MultiByte_Selective_Read(I2C_HandleTypeDef *hi2c, uint32_t mem_addr, uint8_t *pData, uint16_t size)
{
  uint32_t status;
  uint8_t device_addr_write, device_addr_read;
  
  //configure slave address, page select bit and read/write# bit
  if(mem_addr >= 0x10000){ //memory address points to 10000h to 1FFFFh
    device_addr_write = EEPROM_SLAVE_DEV0_ADDR | EEPROM_PAGE_SELECT_H | EEPROM_WRITE_TO_SLAVE;  
    device_addr_read = EEPROM_SLAVE_DEV0_ADDR | EEPROM_PAGE_SELECT_H | EEPROM_READ_FROM_SLAVE;
  }else{ //memory address points to 00000h to 0FFFFh
    device_addr_write = EEPROM_SLAVE_DEV0_ADDR | EEPROM_PAGE_SELECT_L | EEPROM_WRITE_TO_SLAVE;  
    device_addr_read = EEPROM_SLAVE_DEV0_ADDR | EEPROM_PAGE_SELECT_L | EEPROM_READ_FROM_SLAVE;
  }
  
  if(hi2c->State == HAL_I2C_STATE_READY){
    if(__HAL_I2C_GET_FLAG(hi2c, I2C_FLAG_BUSY) == SET)
    {
      return HAL_BUSY;
    }
    
    /* Process Locked */
    __HAL_LOCK(hi2c);

    hi2c->State = HAL_I2C_STATE_BUSY_TX;
    hi2c->ErrorCode = HAL_I2C_ERROR_NONE;

    /* Send Slave Address */
    if(I2C_MasterRequestWrite(hi2c, device_addr_write, 1) != HAL_OK)
    {
      if(hi2c->ErrorCode == HAL_I2C_ERROR_AF)
      {
        /* Process Unlocked */
        __HAL_UNLOCK(hi2c);
        return HAL_ERROR;
      }
      else
      {
        /* Process Unlocked */
        __HAL_UNLOCK(hi2c);
        return HAL_TIMEOUT;
      }
    }

    /* Clear ADDR flag */
    __HAL_I2C_CLEAR_ADDRFLAG(hi2c);
    
    uint32_t Size = 2, i = 0;
    uint8_t memory_addr[] = {
      mem_addr>>8, //memory address MSB
      mem_addr&0x00FF, //memory address LSB
    };
    while(Size > 0)
    {
      /* Wait until TXE flag is set */
      if(I2C_WaitOnFlagUntilTimeout(hi2c, I2C_FLAG_TXE, RESET, 1) != HAL_OK)
      {
        return HAL_TIMEOUT;
      }

      /* Write data to DR */
      hi2c->Instance->DR = memory_addr[i++];
      Size--;

      if((__HAL_I2C_GET_FLAG(hi2c, I2C_FLAG_BTF) == SET) && (Size != 0))
      {
        /* Write data to DR */
        hi2c->Instance->DR = memory_addr[i++];
        Size--;
      }
    }

    /* Wait until TXE flag is set */
    if(I2C_WaitOnFlagUntilTimeout(hi2c, I2C_FLAG_TXE, RESET, 1) != HAL_OK)
    {
      return HAL_TIMEOUT;
    }
    
    /* Continue with Read function */
    hi2c->State = HAL_I2C_STATE_BUSY_RX;
    hi2c->ErrorCode = HAL_I2C_ERROR_NONE;

    /* Send Slave Address */
    if(I2C1_MasterRequestRead(hi2c, device_addr_read, 1) != HAL_OK)
    {
      if(hi2c->ErrorCode == HAL_I2C_ERROR_AF)
      {
        /* Process Unlocked */
        __HAL_UNLOCK(hi2c);
        return HAL_ERROR;
      }
      else
      {
        /* Process Unlocked */
        __HAL_UNLOCK(hi2c);
        return HAL_TIMEOUT;
      }
    }
    
    if(size == 1)
    {
      /* Disable Acknowledge */
      hi2c->Instance->CR1 &= ~I2C_CR1_ACK;

      /* Clear ADDR flag */
      __HAL_I2C_CLEAR_ADDRFLAG(hi2c);

      /* Generate Stop */
      hi2c->Instance->CR1 |= I2C_CR1_STOP;
    }
    else if(size == 2)
    {
      /* Disable Acknowledge */
      hi2c->Instance->CR1 &= ~I2C_CR1_ACK;

      /* Enable Pos */
      hi2c->Instance->CR1 |= I2C_CR1_POS;

      /* Clear ADDR flag */
      __HAL_I2C_CLEAR_ADDRFLAG(hi2c);
    }
    else
    {
      /* Enable Acknowledge */
      hi2c->Instance->CR1 |= I2C_CR1_ACK;

      /* Clear ADDR flag */
      __HAL_I2C_CLEAR_ADDRFLAG(hi2c);
    }

    while(size > 0)
    {
      if(size <= 3)
      {
        /* One byte */
        if(size == 1)
        {
          /* Wait until RXNE flag is set */
          if(I2C_WaitOnFlagUntilTimeout(hi2c, I2C_FLAG_RXNE, RESET, 1) != HAL_OK)
          {
            return HAL_TIMEOUT;
          }

          /* Read data from DR */
          (*pData++) = hi2c->Instance->DR;
          size--;
        }
        /* Two bytes */
        else if(size == 2)
        {
          /* Wait until BTF flag is set */
          if(I2C_WaitOnFlagUntilTimeout(hi2c, I2C_FLAG_BTF, RESET, 1) != HAL_OK)
          {
            return HAL_TIMEOUT;
          }

          /* Generate Stop */
          hi2c->Instance->CR1 |= I2C_CR1_STOP;

          /* Read data from DR */
          (*pData++) = hi2c->Instance->DR;
          size--;

          /* Read data from DR */
          (*pData++) = hi2c->Instance->DR;
          size--;
        }
        /* 3 Last bytes */
        else
        {
          /* Wait until BTF flag is set */
          if(I2C_WaitOnFlagUntilTimeout(hi2c, I2C_FLAG_BTF, RESET, 1) != HAL_OK)
          {
            return HAL_TIMEOUT;
          }

          /* Disable Acknowledge */
          hi2c->Instance->CR1 &= ~I2C_CR1_ACK;

          /* Read data from DR */
          (*pData++) = hi2c->Instance->DR;
          size--;

          /* Wait until BTF flag is set */
          if(I2C_WaitOnFlagUntilTimeout(hi2c, I2C_FLAG_BTF, RESET, 1) != HAL_OK)
          {
            return HAL_TIMEOUT;
          }

          /* Generate Stop */
          hi2c->Instance->CR1 |= I2C_CR1_STOP;

          /* Read data from DR */
          (*pData++) = hi2c->Instance->DR;
          size--;

          /* Read data from DR */
          (*pData++) = hi2c->Instance->DR;
          size--;
        }
      }
      else
      {
        /* Wait until RXNE flag is set */
        if(I2C_WaitOnFlagUntilTimeout(hi2c, I2C_FLAG_RXNE, RESET, 1) != HAL_OK)
        {
          return HAL_TIMEOUT;
        }

        /* Read data from DR */
        (*pData++) = hi2c->Instance->DR;
        size--;

        if(__HAL_I2C_GET_FLAG(hi2c, I2C_FLAG_BTF) == SET)
        {
          /* Read data from DR */
          (*pData++) = hi2c->Instance->DR;
          size--;
        }
      }
    }

    /* Disable Pos */
    hi2c->Instance->CR1 &= ~I2C_CR1_POS;

    /* Wait until BUSY flag is reset */
    if(I2C_WaitOnFlagUntilTimeout(hi2c, I2C_FLAG_BUSY, SET, 1) != HAL_OK)
    {
      return HAL_TIMEOUT;
    }

    hi2c->State = HAL_I2C_STATE_READY;

    /* Process Unlocked */
    __HAL_UNLOCK(hi2c);
    
    
    
    status = HAL_OK;
  }else{
    status = HAL_BUSY;
  }
  
  return status;
}

/**
  * @param  hi2c: pointer to a I2C_HandleTypeDef structure that contains
  *         the configuration information for I2C module
  * @param  DevAddress: Target device address
  * @retval HAL status
  */
static HAL_StatusTypeDef I2C_MasterRequestWrite(I2C_HandleTypeDef *hi2c, uint16_t DevAddress, uint32_t Timeout)
{
  /* Generate Start */
  hi2c->Instance->CR1 |= I2C_CR1_START;

  /* Wait until SB flag is set */
  if(I2C_WaitOnFlagUntilTimeout(hi2c, I2C_FLAG_SB, RESET, Timeout) != HAL_OK)
  {
    return HAL_TIMEOUT;
  }

  if(hi2c->Init.AddressingMode == I2C_ADDRESSINGMODE_7BIT)
  {
    /* Send slave address */
    hi2c->Instance->DR = __HAL_I2C_7BIT_ADD_WRITE(DevAddress);
  }
  else
  {
    /* Send header of slave address */
    hi2c->Instance->DR = __HAL_I2C_10BIT_HEADER_WRITE(DevAddress);

    /* Wait until ADD10 flag is set */
    if(I2C_WaitOnMasterAddressFlagUntilTimeout(hi2c, I2C_FLAG_ADD10, Timeout) != HAL_OK)
    {
      if(hi2c->ErrorCode == HAL_I2C_ERROR_AF)
      {
        return HAL_ERROR;
      }
      else
      {
        return HAL_TIMEOUT;
      }
    }

    /* Send slave address */
    hi2c->Instance->DR = __HAL_I2C_10BIT_ADDRESS(DevAddress);
  }

  /* Wait until ADDR flag is set */
  if(I2C_WaitOnMasterAddressFlagUntilTimeout(hi2c, I2C_FLAG_ADDR, Timeout) != HAL_OK)
  {
    if(hi2c->ErrorCode == HAL_I2C_ERROR_AF)
    {
      return HAL_ERROR;
    }
    else
    {
      return HAL_TIMEOUT;
    }
  }

  return HAL_OK;
}

/**
  * @brief  This function handles I2C Communication Timeout.
  * @param  hi2c: pointer to a I2C_HandleTypeDef structure that contains
  *         the configuration information for I2C module
  * @param  Flag: specifies the I2C flag to check.
  * @param  Status: The new Flag status (SET or RESET).
  * @param  Timeout: Timeout duration
  * @retval HAL status
  */
static HAL_StatusTypeDef I2C_WaitOnFlagUntilTimeout(I2C_HandleTypeDef *hi2c, uint32_t Flag, FlagStatus Status, uint32_t Timeout)
{
  uint32_t tickstart = 0;

  /* Get tick */
  tickstart = HAL_GetTick();

  /* Wait until flag is set */
  if(Status == RESET)
  {
    while(__HAL_I2C_GET_FLAG(hi2c, Flag) == RESET)
    {
      /* Check for the Timeout */
      if(Timeout != HAL_MAX_DELAY)
      {
        if((Timeout == 0)||((HAL_GetTick() - tickstart ) > Timeout))
        {
          hi2c->State= HAL_I2C_STATE_READY;

          /* Process Unlocked */
          __HAL_UNLOCK(hi2c);

          return HAL_TIMEOUT;
        }
      }
    }
  }
  else
  {
    while(__HAL_I2C_GET_FLAG(hi2c, Flag) != RESET)
    {
      /* Check for the Timeout */
      if(Timeout != HAL_MAX_DELAY)
      {
        if((Timeout == 0)||((HAL_GetTick() - tickstart ) > Timeout))
        {
          hi2c->State= HAL_I2C_STATE_READY;

          /* Process Unlocked */
          __HAL_UNLOCK(hi2c);

          return HAL_TIMEOUT;
        }
      }
    }
  }
  return HAL_OK;
}

/**
  * @brief  This function handles I2C Communication Timeout for Master addressing phase.
  * @param  hi2c: pointer to a I2C_HandleTypeDef structure that contains
  *         the configuration information for I2C module
  * @param  Flag: specifies the I2C flag to check.
  * @param  Timeout: Timeout duration
  * @retval HAL status
  */
static HAL_StatusTypeDef I2C_WaitOnMasterAddressFlagUntilTimeout(I2C_HandleTypeDef *hi2c, uint32_t Flag, uint32_t Timeout)
{
  uint32_t tickstart = 0;

  /* Get tick */
  tickstart = HAL_GetTick();

  while(__HAL_I2C_GET_FLAG(hi2c, Flag) == RESET)
  {
    if(__HAL_I2C_GET_FLAG(hi2c, I2C_FLAG_AF) == SET)
    {
      /* Generate Stop */
      hi2c->Instance->CR1 |= I2C_CR1_STOP;

      /* Clear AF Flag */
      __HAL_I2C_CLEAR_FLAG(hi2c, I2C_FLAG_AF);

      hi2c->ErrorCode = HAL_I2C_ERROR_AF;
      hi2c->State= HAL_I2C_STATE_READY;

      /* Process Unlocked */
      __HAL_UNLOCK(hi2c);

      return HAL_ERROR;
    }

    /* Check for the Timeout */
    if(Timeout != HAL_MAX_DELAY)
    {
      if((Timeout == 0)||((HAL_GetTick() - tickstart ) > Timeout))
      {
        hi2c->State= HAL_I2C_STATE_READY;

        /* Process Unlocked */
        __HAL_UNLOCK(hi2c);

        return HAL_TIMEOUT;
      }
    }
  }
  return HAL_OK;
}

/**
  * @brief  Master sends target device address for read request.
  * @param  hi2c: pointer to a I2C_HandleTypeDef structure that contains
  *         the configuration information for I2C module
  * @param  DevAddress: Target device address
  * @retval HAL status
  */
static HAL_StatusTypeDef I2C1_MasterRequestRead(I2C_HandleTypeDef *hi2c, uint16_t DevAddress, uint32_t Timeout)
{
  /* Enable Acknowledge */
  hi2c->Instance->CR1 |= I2C_CR1_ACK;

  /* Generate Start */
  hi2c->Instance->CR1 |= I2C_CR1_START;

  /* Wait until SB flag is set */
  if(I2C_WaitOnFlagUntilTimeout(hi2c, I2C_FLAG_SB, RESET, Timeout) != HAL_OK)
  {
    return HAL_TIMEOUT;
  }

  if(hi2c->Init.AddressingMode == I2C_ADDRESSINGMODE_7BIT)
  {
    /* Send slave address */
    hi2c->Instance->DR = __HAL_I2C_7BIT_ADD_READ(DevAddress);
  }
  else
  {
    /* Send header of slave address */
    hi2c->Instance->DR = __HAL_I2C_10BIT_HEADER_WRITE(DevAddress);

    /* Wait until ADD10 flag is set */
    if(I2C_WaitOnMasterAddressFlagUntilTimeout(hi2c, I2C_FLAG_ADD10, Timeout) != HAL_OK)
    {
      if(hi2c->ErrorCode == HAL_I2C_ERROR_AF)
      {
        return HAL_ERROR;
      }
      else
      {
        return HAL_TIMEOUT;
      }
    }

    /* Send slave address */
    hi2c->Instance->DR = __HAL_I2C_10BIT_ADDRESS(DevAddress);

    /* Wait until ADDR flag is set */
    if(I2C_WaitOnMasterAddressFlagUntilTimeout(hi2c, I2C_FLAG_ADDR, Timeout) != HAL_OK)
    {
      if(hi2c->ErrorCode == HAL_I2C_ERROR_AF)
      {
        return HAL_ERROR;
      }
      else
      {
        return HAL_TIMEOUT;
      }
    }

    /* Clear ADDR flag */
    __HAL_I2C_CLEAR_ADDRFLAG(hi2c);

    /* Generate Restart */
    hi2c->Instance->CR1 |= I2C_CR1_START;

    /* Wait until SB flag is set */
    if(I2C_WaitOnFlagUntilTimeout(hi2c, I2C_FLAG_SB, RESET, Timeout) != HAL_OK)
    {
      return HAL_TIMEOUT;
    }

    /* Send header of slave address */
    hi2c->Instance->DR = __HAL_I2C_10BIT_HEADER_READ(DevAddress);
  }

  /* Wait until ADDR flag is set */
  if(I2C_WaitOnMasterAddressFlagUntilTimeout(hi2c, I2C_FLAG_ADDR, Timeout) != HAL_OK)
  {
    if(hi2c->ErrorCode == HAL_I2C_ERROR_AF)
    {
      return HAL_ERROR;
    }
    else
    {
      return HAL_TIMEOUT;
    }
  }

  return HAL_OK;
}

