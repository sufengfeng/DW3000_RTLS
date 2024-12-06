/* Includes ------------------------------------------------------------------*/
#include <string.h>
#include <stdio.h>
#include "instance.h"
#include "cw2015.h"
#include "i2c.h"
#if defined(L151_DEV)
#include "stm32l1xx_hal.h"
#define IIC_PORT  hi2c2
#else
#include "stm32f1xx_hal.h"
#define IIC_PORT  hi2c1
#endif


uint8_t USE_CW2015 = 0;

void cw2015_write_byte(uint8_t reg, uint8_t data)
{
	HAL_I2C_Mem_Write(&IIC_PORT, CW2015_ADDR, reg, I2C_MEMADD_SIZE_8BIT, &data, 1, 1000);
}

void cw2015_read_byte(uint8_t reg, uint8_t* data)
{
  HAL_I2C_Mem_Read(&IIC_PORT, CW2015_ADDR, reg, I2C_MEMADD_SIZE_8BIT, data, 1, 1000);
}

void cw2015_wakeup(void)
{
  cw2015_write_byte(REG_MODE, 0x00);
}

void cw2015_sleep(void)
{
  cw2015_write_byte(REG_MODE, 0xC0);
}

uint8_t cw2015_read_battery(void)
{
  uint8_t cw2015_soc;
  cw2015_read_byte(REG_SOC, &cw2015_soc);
  //printf("cw2015_soc = %d\r\n", cw2015_soc);
  return cw2015_soc;
}


uint8_t cw2015_init(void)
{
  uint8_t cw2015_ver;
  cw2015_wakeup();
  HAL_Delay(50);
  cw2015_read_byte(REG_VERSION, &cw2015_ver);
  if(cw2015_ver != 0x73)
    return 0;
  
  return 1;
}



