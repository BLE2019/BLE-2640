#include "GT20L16S1Y.h"
#include "hw_spi.h"
#include "board.h"
void GT20L_Init(void)
{
    HwSPIInit();
 }
uint32_t GT20L_Addr(uint8_t *hanzi)        //该函数将内码作为参数，计算出汉字在字库芯片中的地址
{
  uint8_t MSB,LSB;
  uint32_t Address=0;
  uint32_t BaseAdd=0;
        MSB = hanzi[0];
        LSB = hanzi[1];
        if(MSB ==0xA9 && LSB >=0xA1)   
          {
            Address = (282+(LSB-0xA1))*32 + BaseAdd;
          }
        else if(MSB >=0xA1 && MSB <= 0xA3 && LSB >=0xA1)   
          {
            Address =( (MSB - 0xA1) * 94 + (LSB - 0xA1))*32+ BaseAdd;
          }
        else if(MSB >=0xB0 && MSB <= 0xF7 && LSB >=0xA1)   
          {
            Address = ((MSB - 0xB0) * 94 + (LSB - 0xA1)+846)*32+BaseAdd;
          }
        
       return Address;
}
void get_hanziaddr(uint8_t *hanzi,uint8_t *r_buffer)
{
  uint8_t MSB,LSB,lsb,msb,ssb,i;
  uint32_t Address=0;
  uint32_t BaseAdd=0;
  uint8_t t_buffer[4];
  uint8_t r_buffer1[36];
        MSB = hanzi[0];
        LSB = hanzi[1];
        if(MSB ==0xA9 && LSB >=0xA1)   
          {
            Address = (282+(LSB-0xA1))*32 + BaseAdd;
          }
        else if(MSB >=0xA1 && MSB <= 0xA3 && LSB >=0xA1)   
          {
            Address =( (MSB - 0xA1) * 94 + (LSB - 0xA1))*32+ BaseAdd;
          }
        else if(MSB >=0xB0 && MSB <= 0xF7 && LSB >=0xA1)   
          {
            Address = ((MSB - 0xB0) * 94 + (LSB - 0xA1)+846)*32+BaseAdd;
          }
        
   lsb=(Address&0xff0000)>>16;
   msb=(Address&0xff00)>>8;
   ssb=(Address&0xff);
   //快速获得汉字命令
   for(i=0;i<4;i++)
   {
   t_buffer[0]=0x03;
   t_buffer[1]=lsb;
   t_buffer[2]=msb;
   t_buffer[3]=ssb;
   //t_buffer[4]=0xff;
   }
  //OLED_Delay(65535); 
 HwSPITrans(Board_SPI_FLASH_CSN, t_buffer, r_buffer1,36);
  for(i=0;i<32;i++)
 {
    r_buffer[i]=r_buffer1[i+4];
}
}