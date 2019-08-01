#include "TFT_touch.h"
#include "board.h"
#include "hw_gpio.h"
#include "hw_spi.h"
#include "LCDFONT.h"
struct tp_pix_  tp_pixad,tp_pixlcd;	 //当前触控坐标的AD值，前触控坐标的像素值  
 //定义LCD的尺寸	
#define LCD_W 128
#define LCD_H 160
//定义LCD背景字体颜色
uint16_t BACK_COLOR=WHITE;//背景色
uint16_t POINT_COLOR=RED;//字体色
/*
TFT_SPI_RESET（IOID_0）   RES
Board_LCD_MODE（IOID_1）   DC
Board_LCD_CSN（IOID_11）  CS1
*/
void TFT_delay(uint32_t times)
{
    while(times > 0) {
        times --;
    }
}
//置位RESET
static inline void TFT_SetRes()
{
    HwGPIOSet(Board_3V3_EN, 1);
}
//清除RESET
static inline void TFT_ClearRes()
{
    HwGPIOSet(Board_3V3_EN, 0);
}
//写8位数据
void LCD_WR_DATA8(uint8_t TFT_txbuff)
{
  uint8_t TFT_rxbuf_data[1],TFT_txbuf_data[1];
  TFT_txbuf_data[0]=TFT_txbuff;
 HwGPIOSet(Board_LCD_MODE,1);//写数据
 HwSPITrans(Board_LCD_CSN, TFT_txbuf_data, TFT_rxbuf_data ,1);
}
//写寄存器       
void LCD_WR_REG(uint8_t TFT_txbuff1)
{
 uint8_t TFT_rxbuf_reg[1],TFT_txbuf_reg[1];
 TFT_txbuf_reg[0]=TFT_txbuff1;
 HwGPIOSet(Board_LCD_MODE,0);//写寄存器
 HwSPITrans(Board_LCD_CSN, TFT_txbuf_reg, TFT_rxbuf_reg ,1);
}
//写16位数据      
void LCD_WR_DATA16(uint16_t TFT_txbuff2)
{
  uint8_t TFT_rxbuf_data[2],TFT_txbuf_data[2];
  TFT_txbuf_data[0]=TFT_txbuff2>>8;
  TFT_txbuf_data[1]=TFT_txbuff2;
  HwGPIOSet(Board_LCD_MODE,1);//写16位数据
  HwSPITrans(Board_LCD_CSN, TFT_txbuf_data, TFT_rxbuf_data ,2);
}
//TFT初始化   
void TFT_Init(void)
{
        HwGPIOInit();//端口初始化
	TFT_ClearRes();
	TFT_delay(100);
	TFT_SetRes();
	TFT_delay(100);
	//HwGPIOSet(Board_GLED,0);//背光关闭
//************* Start Initial Sequence **********// 
        LCD_WR_REG(0xB1); 
	LCD_WR_DATA8(0x01); 
	LCD_WR_DATA8(0x2C); 
	LCD_WR_DATA8(0x2D); 

	LCD_WR_REG(0xB2); 
	LCD_WR_DATA8(0x01); 
	LCD_WR_DATA8(0x2C); 
	LCD_WR_DATA8(0x2D); 

	LCD_WR_REG(0xB3); 
	LCD_WR_DATA8(0x01); 
	LCD_WR_DATA8(0x2C); 
	LCD_WR_DATA8(0x2D); 
	LCD_WR_DATA8(0x01); 
	LCD_WR_DATA8(0x2C); 
	LCD_WR_DATA8(0x2D); 
	
	LCD_WR_REG(0xB4); //Column inversion 
	LCD_WR_DATA8(0x07); 
	
	//ST7735R Power Sequence
	LCD_WR_REG(0xC0); 
	LCD_WR_DATA8(0xA2); 
	LCD_WR_DATA8(0x02); 
	LCD_WR_DATA8(0x84); 
	LCD_WR_REG(0xC1); 
	LCD_WR_DATA8(0xC5); 

	LCD_WR_REG(0xC2); 
	LCD_WR_DATA8(0x0A); 
	LCD_WR_DATA8(0x00); 

	LCD_WR_REG(0xC3); 
	LCD_WR_DATA8(0x8A); 
	LCD_WR_DATA8(0x2A); 
	LCD_WR_REG(0xC4); 
	LCD_WR_DATA8(0x8A); 
	LCD_WR_DATA8(0xEE); 
	
	LCD_WR_REG(0xC5); //VCOM 
	LCD_WR_DATA8(0x0E); 
	
		LCD_WR_REG(0x36); //MX, MY, RGB mode 
	LCD_WR_DATA8(0xc0); 
 
LCD_WR_REG(0x3A);   
LCD_WR_DATA8(0x55); 

LCD_WR_REG(0xB1);   
LCD_WR_DATA8(0x00);   
LCD_WR_DATA8(0x18); 
 
LCD_WR_REG(0xB6);    // Display Function Control 
LCD_WR_DATA8(0x0A); 
LCD_WR_DATA8(0xA2); 

 
 
LCD_WR_REG(0xF2);    // 3Gamma Function Disable 
LCD_WR_DATA8(0x00); 
 
LCD_WR_REG(0x26);    //Gamma curve selected 
LCD_WR_DATA8(0x01); 
 
LCD_WR_REG(0xE0);    //Set Gamma 
LCD_WR_DATA8(0x0F); 
LCD_WR_DATA8(0x20); 
LCD_WR_DATA8(0x1E); 
LCD_WR_DATA8(0x09); 
LCD_WR_DATA8(0x12); 
LCD_WR_DATA8(0x0B); 
LCD_WR_DATA8(0x50); 
LCD_WR_DATA8(0XBA); 
LCD_WR_DATA8(0x44); 
LCD_WR_DATA8(0x09); 
LCD_WR_DATA8(0x14); 
LCD_WR_DATA8(0x05); 
LCD_WR_DATA8(0x23); 
LCD_WR_DATA8(0x21); 
LCD_WR_DATA8(0x00); 
 
LCD_WR_REG(0XE1);    //Set Gamma 
LCD_WR_DATA8(0x00); 
LCD_WR_DATA8(0x19); 
LCD_WR_DATA8(0x19); 
LCD_WR_DATA8(0x00); 
LCD_WR_DATA8(0x12); 
LCD_WR_DATA8(0x07); 
LCD_WR_DATA8(0x2D); 
LCD_WR_DATA8(0x28); 
LCD_WR_DATA8(0x3F); 
LCD_WR_DATA8(0x02); 
LCD_WR_DATA8(0x0A); 
LCD_WR_DATA8(0x08); 
LCD_WR_DATA8(0x25); 
LCD_WR_DATA8(0x2D); 
LCD_WR_DATA8(0x0F); 
 
LCD_WR_REG(0x11);    //Exit Sleep 
TFT_delay(120); 
LCD_WR_REG(0x29);    //Display on 
TFT_Clear(WHITE);//白色背景
} 
//设置坐标
void Address_set(uint16_t x1,uint16_t y1,uint16_t x2,uint16_t y2)
{ 
//   LCD_WR_REG(0x2a);//列地址设置
//   LCD_WR_DATA8(0x00);
//   LCD_WR_DATA8(x1+2);
//   LCD_WR_DATA8(0x00);
//   LCD_WR_DATA8(x2+2);
//  
//   LCD_WR_REG(0x2b);//行地址设置
//   LCD_WR_DATA8(0x00);
//   LCD_WR_DATA8(y1+1);
//   LCD_WR_DATA8(0x00);
//   LCD_WR_DATA8(y2+1);
//
//   LCD_WR_REG(0x2C);	//储存器写
   		LCD_WR_REG(0x2a);//列地址设置
		LCD_WR_DATA16(x1+2);
		LCD_WR_DATA16(x2+2);
		LCD_WR_REG(0x2b);//行地址设置
		LCD_WR_DATA16(y1+1);
		LCD_WR_DATA16(y2+1);
		LCD_WR_REG(0x2c);//储存器写
   

}
//清屏函数
//Color:要清屏的填充色
void TFT_Clear(uint16_t Color)
{
	uint16_t i,j;  	
	Address_set(0,0,LCD_W-1,LCD_H-1);
        for(i=0;i<LCD_W;i++)
	 {
	      for (j=0;j<LCD_H;j++)
	   	{
        	   LCD_WR_DATA16(Color);
	        }
	 }
}
//指定位置填充颜色
void LCD_Fill(uint16_t xsta,uint16_t ysta,uint16_t xend,uint16_t yend,uint16_t color)
{          
	uint16_t i,j; 
	Address_set(xsta,ysta,xend,yend);      //设置光标位置
	for(i=ysta;i<=yend;i++)
	{													   	 	
		for(j=xsta;j<=xend;j++)
                { 
                  LCD_WR_DATA16(color);//设置光标位置	
                }   
	} 					  	    
} 
//写字符
void LCD_ShowChar(uint16_t x,uint16_t y,uint8_t num,uint8_t mode)
{
    uint8_t temp;
    uint8_t pos,t;
	//uint16_t x0=x;
	uint16_t colortemp=POINT_COLOR;      
    if(x>LCD_W-16||y>LCD_H-16)return;	    
	//设置窗口	   
	num=num-' ';//得到偏移后的值
	Address_set(x,y,x+8-1,y+16-1);      //设置光标位置
	if(!mode) //非叠加方式
	{
		for(pos=0;pos<16;pos++)
		{ 
			temp=asc2_1608[(uint16_t)num*16+pos];		 //调用1608字体
			for(t=0;t<8;t++)
		    {                 
		        if(temp&0x01)POINT_COLOR=colortemp;
				else POINT_COLOR=BACK_COLOR;
				LCD_WR_DATA16(POINT_COLOR);	
				temp>>=1; 
				//x++;
		    }
			//x=x0;
			//y++;
		}	
	}
        else//叠加方式
	{
		for(pos=0;pos<16;pos++)
		{
		    temp=asc2_1608[(uint16_t)num*16+pos];		 //调用1608字体
			for(t=0;t<8;t++)
		    {                 
		        if(temp&0x01)LCD_DrawPoint(x+t,y+pos);//画一个点   
		        temp>>=1; 
		    }
		}
	}
        POINT_COLOR=colortemp;	
}   
//画点
//POINT_COLOR:此点的颜色
void LCD_DrawPoint(uint16_t x,uint16_t y)
{
	Address_set(x,y,x,y);//设置光标位置
	LCD_WR_DATA16(POINT_COLOR); 	    
} 
//显示字符串
//x,y:起点坐标 
//*p:字符串起始地址
//用16字体
void LCD_ShowString(uint16_t x,uint16_t y,uint8_t *p)
{         
    while(*p!='\0')
    {       
        if(x>LCD_W-16){x=0;y+=16;}
        if(y>LCD_H-16){y=x=0;TFT_Clear(RED);}
        LCD_ShowChar(x,y,*p,0);
        x+=8;
        p++;
    }  
}
//在指定位置生成一个汉字(16*16大小)
void showhanzi16(uint16_t x,uint16_t y,uint8_t index)	
{  
	uint8_t i,j;
	uint8_t *temp;   
        uint8_t ziku[32];
        Address_set(x,y,x+15,y+15); //设置区域  
        for(i=0;i<16;i++)
        {
          ziku[2*i]=GB20L16S1Y[15-i];
          ziku[2*i+1]=GB20L16S1Y[31-i];
        }
        temp=ziku;
	for(j=0;j<32;j++)
	{
		for(i=0;i<8;i++)
		{ 		     
		 	if((*temp&(1<<i))!=0)
			{
				LCD_WR_DATA16(POINT_COLOR);
			} 
			else
			{
				LCD_WR_DATA16(BACK_COLOR);
			}   
		}
		temp++;
	 }
}

/********************************************************************************
下面为触摸屏函数
Board_SPI_FLASH_CSN 为cs2
********************************************************************************/
//读spi//Board_RLED为触摸屏片选引脚
void touch_WR_DATA8(uint8_t touch_txbuff)
{
  uint8_t touch_rxbuf_data[1],touch_txbuf_data[1];
  touch_rxbuf_data[0]=touch_txbuff;
 HwSPITrans(Board_SPI_FLASH_CSN, touch_txbuf_data, touch_rxbuf_data ,1);

}
uint16_t touch_Read_12bit()             //SPI读数据
{
	uint16_t Num=0;
  uint8_t touch_rxbuf[2];
  uint8_t touch_txbuf[2]={0x00,0x00};

   HwSPITrans(Board_SPI_FLASH_CSN, touch_txbuf, touch_rxbuf ,2);
   Num=((touch_rxbuf[0]&0x78)>>3)*256+((touch_rxbuf[1]&0xf1)>>3)+(touch_rxbuf[0]<<5);
return Num;
}	
//从XPT2046读ADC值	  0x90=y   0xd0=x
uint16_t ADS_Read_AD(uint8_t CMD)      //需要24个时钟周期    
{
uint16_t l;
TFT_delay(8);//延时一个spi时钟
touch_WR_DATA8(CMD);//送控制字即用差分方式读x坐标值
  //LCD_SCK=1; _nop_();_nop_();_nop_();_nop_();
  //LCD_SCK=0; _nop_();_nop_();_nop_();_nop_();
l=touch_Read_12bit();
return l;
}
//读取一个坐标值
//连续读取 READ_TIMES次数据
#define READ_TIMES 15 //读取次数
#define LOST_VAL 5	  //丢弃值
uint16_t ADS_Read_XY(uint8_t xy)
{
	uint16_t i, j;
	uint16_t buf[READ_TIMES];
	uint16_t sum=0;
	uint16_t temp;
	for(i=0;i<READ_TIMES;i++)
	{				 
		buf[i]=ADS_Read_AD(xy);	    
	}				    
	for(i=0;i<READ_TIMES-1; i++)//排序
	{
		for(j=i+1;j<READ_TIMES;j++)
		{
			if(buf[i]>buf[j])//升序排列
			{
				temp=buf[i];
				buf[i]=buf[j];
				buf[j]=temp;
			}
		}
	}	  
	sum=0;
	for(i=LOST_VAL;i<READ_TIMES-LOST_VAL;i++)sum+=buf[i];
	temp=sum/(READ_TIMES-2*LOST_VAL);
	return temp;   
} 
//带滤波的坐标读取
//最小值不能小于100
uint8_t Read_ADS(uint16_t *x,uint16_t *y)
{
	uint16_t xtemp,ytemp;			 	 		  
	xtemp=ADS_Read_XY(CMD_RDX);
	ytemp=ADS_Read_XY(CMD_RDY);	 									   
	if(xtemp<100||ytemp<100)return 0;//读数失败
	*x=xtemp;
	*y=ytemp;
	return 1;//读数成功
}

//连续两次读取有效的AD值，且两次的偏差不能超过ERR_RANGE值，满足条件则读值正确
#define ERR_RANGE 100 //误差范围
uint8_t Read_ADS2(uint16_t *x,uint16_t *y) 
{
	uint16_t x1,y1;
 	uint16_t x2,y2;
 	uint8_t flag;    
    flag=Read_ADS(&x1,&y1);   
    if(flag==0)return(0);
    flag=Read_ADS(&x2,&y2);	
    if(flag==0)return(0);   
    if(((x2<=x1&&x1<x2+ERR_RANGE)||(x1<=x2&&x2<x1+ERR_RANGE))//前两次采样在+-ERR_RANGE
    &&((y2<=y1&&y1<y2+ERR_RANGE)||(y1<=y2&&y2<y1+ERR_RANGE)))
    {
        *x=(x1+x2)>>1;
        *y=(y1+y2)>>1;		
        return 1;
    }else return 0;	  
}
//精确读取一次坐标，校准的时候用	   
uint8_t Read_TP_Once(void)
{
	uint8_t re=0;
	uint16_t x1,y1;
	while(re==0)
	{
		while(!Read_ADS2(&tp_pixad.x,&tp_pixad.y));
		TFT_delay(1000);
		while(!Read_ADS2(&x1,&y1));
		if(tp_pixad.x==x1&&tp_pixad.y==y1)
		{
			re=1; 
		}
	} 
	return re;
}