#ifndef __MAX31865_H
#define __MAX31865_H

#include <stdint.h>
#include "stm32f1xx.h"
#include "stm32f1xx_hal_conf.h"
#include "system_stm32f1xx.h"
#include "main.h"

#define SDI_H HAL_GPIO_WritePin(GPIOC,GPIO_PIN_2,GPIO_PIN_SET)
#define SDI_L HAL_GPIO_WritePin(GPIOC,GPIO_PIN_2,GPIO_PIN_RESET)
#define SCLK_H HAL_GPIO_WritePin(GPIOC,GPIO_PIN_3,GPIO_PIN_SET)
#define SCLK_L HAL_GPIO_WritePin(GPIOC,GPIO_PIN_3,GPIO_PIN_RESET)
#define NCS_H HAL_GPIO_WritePin(GPIOC,GPIO_PIN_4,GPIO_PIN_SET)
#define NCS_L HAL_GPIO_WritePin(GPIOC,GPIO_PIN_4,GPIO_PIN_RESET)
#define NCS2_H HAL_GPIO_WritePin(GPIOC,GPIO_PIN_5,GPIO_PIN_SET)
#define NCS2_L HAL_GPIO_WritePin(GPIOC,GPIO_PIN_5,GPIO_PIN_RESET)

#define SDO HAL_GPIO_ReadPin(GPIOC,GPIO_PIN_0)
#define DRDY HAL_GPIO_ReadPin(GPIOC,GPIO_PIN_1)//iso module is not connect
uint8_t MAX31865_SB_Read(uint8_t addr,uint8_t cs);
void MAX31865_SB_Write(uint8_t addr,uint8_t wdata,uint8_t cs);
float Get_tempture(uint8_t ncs);
void Delay(__IO uint32_t nCount) ;
#endif 