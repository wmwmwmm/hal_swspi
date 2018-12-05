#include "MAX31865.h"
void Delay(__IO uint32_t nCount) 
{ 
  for(; nCount != 0; nCount--); 
} 

uint8_t MAX31865_SB_Read(uint8_t addr,uint8_t cs)//SPI Single-Byte Read
{
  uint8_t read = 0;
  uint8_t i = 0;
  if(cs == 1){
  NCS_L;
  }
  else{NCS2_L;}
  Delay(0x1f);
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
  NCS2_H;
  return read;
}

void MAX31865_SB_Write(uint8_t addr,uint8_t wdata,uint8_t cs)//SPI Single-Byte Write
{
  uint8_t i = 0;
  if(cs == 1){
    NCS_L;}
  else{
    NCS2_L;
  }

        Delay(0x1f);
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
    NCS2_H;
}


float Get_tempture(uint8_t ncs)
{
  float temps;
  uint16_t dtemp0,dtemp1,data_temp1,data_temp2;;
  dtemp0 = dtemp1 = 0 ;
  
  dtemp0=MAX31865_SB_Read(0x01,ncs);
  dtemp1=MAX31865_SB_Read(0x02,ncs);
  data_temp1 = dtemp0*128 ;
  data_temp2 = dtemp1/2;
  //data_temp=(dtemp0<<7)+(dtemp1>>1);//Get 15Bit DATA;
  temps=data_temp1+data_temp2;
  temps=(temps*430)/32768;//Here is the rtd R value;
  temps=(temps-100)/0.385055;//A gruad
  return temps;
}