#ifdef SCANBEACON_ADVANCE

#include "oled.h"
#include "board.h"
#include "hw_gpio.h"
#include "oledfont.h"

void OLED_Delay(uint32_t times)
{
    while(times > 0) {
        times --;
    }
}
//��λRESET

static inline void OLED_SetRes()
{
    HwGPIOSet(OLED_SPI_RESET, 1);
}
//���RESET
static inline void OLED_ClearRes()
{
    HwGPIOSet(OLED_SPI_RESET, 0);
}
//��SSD1106д��һ���ֽ�
void OLED_WR_Byte(uint8_t data,uint8_t cmd)
{
    uint8_t i;
    HwGPIOSet(OLED_SPI_CS,0);
    NOP(6);
    HwGPIOSet(Board_SPI0_CLK,0);
    NOP(6);
    if(cmd)
      HwGPIOSet(Board_SPI0_MOSI,1);
    else
      HwGPIOSet(Board_SPI0_MOSI,0);
    NOP(6);
    HwGPIOSet(Board_SPI0_CLK,1);
    NOP(6);
    for(i=0;i<8;i++)
    {
      HwGPIOSet(Board_SPI0_CLK,0);
      NOP(6);
      if(data&0x80)
        HwGPIOSet(Board_SPI0_MOSI,1);
      else
        HwGPIOSet(Board_SPI0_MOSI,0);
      NOP(6);
      HwGPIOSet(Board_SPI0_CLK,1);
      data<<=1;
      NOP(6);
    }
    HwGPIOSet(OLED_SPI_CS,1);
}

//��������
void OLED_Set_Pos(uint8_t x,uint8_t y)
{
    OLED_WR_Byte(0xb0+y,OLED_CMD);
    OLED_WR_Byte(((x&0xf0)>>4)|0x10,OLED_CMD);
    OLED_WR_Byte((x&0x0f)|0x01,OLED_CMD);
}
//����OLED��ʾ
void OLED_Display_On(void)
{
    OLED_WR_Byte(0X8D,OLED_CMD);  //SET DCDC����
    OLED_WR_Byte(0X14,OLED_CMD);  //DCDC ON
    OLED_WR_Byte(0XAF,OLED_CMD);  //DISPLAY ON
}
//�ر�OLED��ʾ    
void OLED_Display_Off(void)
{
    OLED_WR_Byte(0X8D,OLED_CMD);  //SET DCDC����
    OLED_WR_Byte(0X10,OLED_CMD);  //DCDC OFF
    OLED_WR_Byte(0XAE,OLED_CMD);  //DISPLAY OFF
}
//����	  
void OLED_Clear(void)  
{  
	uint8_t i,n;		    
	for(i=0;i<8;i++)  
	{  
            OLED_WR_Byte (0xb0+i,OLED_CMD);    //����ҳ��ַ��0-7��
            OLED_WR_Byte (0x00,OLED_CMD);      //����ҳ��ʾλ��-�е͵�ַ
            OLED_WR_Byte (0x10,OLED_CMD);      //����ҳ��ʾλ��-�иߵ�ַ  
            for(n=0;n<128;n++)
              OLED_WR_Byte(0,OLED_DATA); 
	} //������ʾ
}
//��ָ��λ����ʾһ���ַ�
//x:0~127
//y:0~63
//mode:0,������ʾ;1,������ʾ				 
//size:ѡ������ 16/12 
void OLED_ShowChar(uint8_t x,uint8_t y,uint8_t chr)
{      	
    unsigned char c=0,i=0;	
    c=chr-' ';//�õ�ƫ�ƺ��ֵ		
    if(x>Max_Column-1){x=0;y=y+2;}
    if(OLED_SIZE ==16)
      {
      OLED_Set_Pos(x,y);	
      for(i=0;i<8;i++)
      OLED_WR_Byte(F8X16[c*16+i],OLED_DATA);
      OLED_Set_Pos(x,y+1);
      for(i=0;i<8;i++)
      OLED_WR_Byte(F8X16[c*16+i+8],OLED_DATA);
      }
      else 
      {	
          OLED_Set_Pos(x,y+1);
          for(i=0;i<6;i++)
          OLED_WR_Byte(F6x8[c][i],OLED_DATA);     
      }
}
void OLED_Init(void)
{
    HwGPIOSet(OLED_POWER_EN,0);
    /* Reset OLED and Waitting for Vcc stable */
    OLED_SetRes();
    OLED_Delay(100);
    OLED_ClearRes();
    OLED_Delay(100);
    OLED_SetRes();
    
    OLED_WR_Byte(0xAE,OLED_CMD);//--turn off oled panel
    OLED_WR_Byte(0x00,OLED_CMD);//---set low column address
    OLED_WR_Byte(0x10,OLED_CMD);//---set high column address
    OLED_WR_Byte(0x40,OLED_CMD);//--set start line address  Set Mapping RAM Display Start Line (0x00~0x3F)
    OLED_WR_Byte(0x81,OLED_CMD);//--set contrast control register
    OLED_WR_Byte(0xCF,OLED_CMD); // Set SEG Output Current Brightness
    OLED_WR_Byte(0xA1,OLED_CMD);//--Set SEG/Column Mapping     0xa0������������?? 0xa1?y3��
    OLED_WR_Byte(0xC8,OLED_CMD);//Set COM/Row Scan Direction   0xc0��???����?? 0xc8?y3��
    OLED_WR_Byte(0xA6,OLED_CMD);//--set normal display
    OLED_WR_Byte(0xA8,OLED_CMD);//--set multiplex ratio(1 to 64)
    OLED_WR_Byte(0x3f,OLED_CMD);//--1/64 duty
    OLED_WR_Byte(0xD3,OLED_CMD);//-set display offset	Shift Mapping RAM Counter (0x00~0x3F)
    OLED_WR_Byte(0x00,OLED_CMD);//-not offset
    OLED_WR_Byte(0xd5,OLED_CMD);//--set display clock divide ratio/oscillator frequency
    OLED_WR_Byte(0x80,OLED_CMD);//--set divide ratio, Set Clock as 100 Frames/Sec
    OLED_WR_Byte(0xD9,OLED_CMD);//--set pre-charge period
    OLED_WR_Byte(0xF1,OLED_CMD);//Set Pre-Charge as 15 Clocks & Discharge as 1 Clock
    OLED_WR_Byte(0xDA,OLED_CMD);//--set com pins hardware configuration
    OLED_WR_Byte(0x12,OLED_CMD);
    OLED_WR_Byte(0xDB,OLED_CMD);//--set vcomh
    OLED_WR_Byte(0x40,OLED_CMD);//Set VCOM Deselect Level
    OLED_WR_Byte(0x20,OLED_CMD);//-Set Page Addressing Mode (0x00/0x01/0x02)
    OLED_WR_Byte(0x02,OLED_CMD);//
    OLED_WR_Byte(0x8D,OLED_CMD);//--set Charge Pump enable/disable
    OLED_WR_Byte(0x14,OLED_CMD);//--set(0x10) disable
    OLED_WR_Byte(0xA4,OLED_CMD);// Disable Entire Display On (0xa4/0xa5)
    OLED_WR_Byte(0xA6,OLED_CMD);// Disable Inverse Display On (0xa6/a7) 
    OLED_WR_Byte(0xAF,OLED_CMD);//--turn on oled panel

    OLED_WR_Byte(0xAF,OLED_CMD); /*display ON*/ 
    OLED_Clear();
    OLED_Set_Pos(0,0);
}
#endif