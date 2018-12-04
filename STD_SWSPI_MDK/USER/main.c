#include "led.h"
#include "delay.h"
#include "sys.h"

#define SDI_H GPIO_SetBits(GPIOC,GPIO_Pin_2)
#define SDI_L GPIO_ResetBits(GPIOC,GPIO_Pin_2)
#define SCLK_H GPIO_SetBits(GPIOC,GPIO_Pin_3)
#define SCLK_L GPIO_ResetBits(GPIOC,GPIO_Pin_3)
#define NCS_H GPIO_SetBits(GPIOC,GPIO_Pin_4)
#define NCS_L GPIO_ResetBits(GPIOC,GPIO_Pin_4)

#define SDO GPIO_ReadInputDataBit(GPIOC,GPIO_Pin_0)
#define DRDY GPIO_ReadInputDataBit(GPIOC,GPIO_Pin_1)//iso module is not connect

uint8_t Fault_Status;  
float tempture;
//ALIENTEK miniSTM32开发板实验1
//跑马灯实验  
//技术支持：www.openedv.com
//广州市星翼电子科技有限公司
void Delay(vu32 nCount) 
{ 
  for(; nCount != 0; nCount--); 
} 

void GPIO_Configuration(void) 
{ 
  GPIO_InitTypeDef GPIO_InitStructure; 
  GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_2|GPIO_Pin_3|GPIO_Pin_4; 
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz; 
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; 
  GPIO_Init(GPIOC, &GPIO_InitStructure); 

  GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_0|GPIO_Pin_1; 
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz; 
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING; 
  GPIO_Init(GPIOC, &GPIO_InitStructure); 
} 


void RCC_Configuration(void) 
{    
  ErrorStatus HSEStartUpStatus;
  RCC_DeInit(); 
  RCC_HSEConfig(RCC_HSE_ON); 
  HSEStartUpStatus = RCC_WaitForHSEStartUp(); 
  if(HSEStartUpStatus == SUCCESS) 
  { 
    FLASH_PrefetchBufferCmd(FLASH_PrefetchBuffer_Enable); 
    FLASH_SetLatency(FLASH_Latency_2); 
    RCC_HCLKConfig(RCC_SYSCLK_Div1);  
    RCC_PCLK2Config(RCC_HCLK_Div1);  
    RCC_PCLK1Config(RCC_HCLK_Div2); 
    RCC_PLLConfig(RCC_PLLSource_HSE_Div1, RCC_PLLMul_9); 
    RCC_PLLCmd(ENABLE); 
    while(RCC_GetFlagStatus(RCC_FLAG_PLLRDY) == RESET) { } 
    RCC_SYSCLKConfig(RCC_SYSCLKSource_PLLCLK); 
    while(RCC_GetSYSCLKSource() != 0x08) { } 
  } 
  //RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA2, ENABLE); 
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE); 
  //RCC_APB1PeriphClockCmd(RCC_APB1Periph_DAC, ENABLE); 
  //RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM6, ENABLE); 
} 


uint8_t MAX31865_SB_Read(uint8_t addr)//SPI Single-Byte Read
{
  uint8_t read = 0;
	uint8_t i = 0;
  NCS_L;Delay(0x1f);
  for(i = 0; i < 8; i++)
  {
    SCLK_H;
    if (addr & 0x80){SDI_H;}
    else {SDI_L;}
    Delay(0x1f);
    SCLK_L;
    addr <<= 1;
    Delay(0x1f);
  }
  i = 0;
  Delay(0x1f);
  for (i = 0; i < 8; i++)
  {
    SCLK_H;
    read = read<<1;
    Delay(0x1f);
    if(SDO)
    {
      read++;
    }
    SCLK_L;
    Delay(0x1f);
  }
  NCS_H;
  return read;
}

void MAX31865_SB_Write(uint8_t addr,uint8_t wdata)//SPI Single-Byte Write
{
  uint8_t i = 0;
	NCS_L;Delay(0x1f);
  for(i = 0; i < 8; i++)
  {
    SCLK_H;
    if (addr & 0x80){SDI_H;}
    else {SDI_L;}
    Delay(0x1f);
    SCLK_L;
    addr <<= 1;
    Delay(0x1f);
  }
  
  for(i = 0; i < 8; i++)
  {
    SCLK_H;
    if (wdata & 0x80){SDI_H;}
    else {SDI_L;}
    Delay(0x1f);
    SCLK_L;
    wdata <<= 1;
    Delay(0x1f);
  }
  NCS_H;
}


float Get_tempture(void)
{
  float temps;
  uint16_t dtemp[2];
  uint16_t data_temp;
  dtemp[0]=MAX31865_SB_Read(0x01);
  dtemp[1]=MAX31865_SB_Read(0x02);
  data_temp=(dtemp[0]<<7)+(dtemp[1]>>1);//Get 15Bit DATA;
  temps=data_temp;
  temps=(temps*430)/32768;//Here is the rtd R value;
  temps=(temps-100)/0.385055;//A gruad
  return temps;
}
 int main(void)
 {	
	RCC_Configuration();  
	
//	GPIO_InitTypeDef GPIO_InitStructure; 
//	 
	//RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);
	 GPIO_Configuration();

	 MAX31865_SB_Write(0x80,0xc1);//MAX31865 init (50Hz filter)
  //MAX31865_SB_Write(0x80,0xFF);//Set High Fault Threshold MSB
  //MAX31865_SB_Write(0x80,0xFF);//Set High Fault Threshold LSB
  //MAX31865_SB_Write(0x80,0x00);//Set Low Fault Threshold MSB 
  //MAX31865_SB_Write(0x80,0x00);//Set Low Fault Threshold LSB 
  Delay(0xfff);
  while (1) 
  { 
    Delay(0x1ffff);
    tempture=Get_tempture();
    Fault_Status=MAX31865_SB_Read(0x07);//Get Fault_Status
  } 
 }

