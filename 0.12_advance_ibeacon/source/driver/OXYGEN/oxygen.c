#include "oxygen.h"
#include <stdint.h>
#include "board.h"
#include "hw_gpio.h"

void Delay(uint32_t times)
{
    while(times > 0) {
        times --;
    }
}
const PIN_Config IIC_SDA_OUT[] =
{
    Board_I2C0_SDA0 | PIN_GPIO_OUTPUT_EN | PIN_GPIO_LOW | PIN_PUSHPULL | PIN_DRVSTR_MAX,
    PIN_TERMINATE
};
const PIN_Config IIC_SDA_IN[] =
{
    Board_I2C0_SDA0 | PIN_INPUT_EN | PIN_GPIO_LOW | PIN_PUSHPULL | PIN_DRVSTR_MAX,
    PIN_TERMINATE
};
//SDA输出模式
static void SDA_OUT(void)
{
    PIN_init(IIC_SDA_OUT);
}
//SDA输入模式
static void SDA_IN(void)
{
  PIN_init(IIC_SDA_IN);
}

//产生IIC起始信号
static void IIC_Start(void)
{
    SDA_OUT();
    HwGPIOSet(Board_I2C0_SDA0,1);
    HwGPIOSet(Board_I2C0_SCL0,1);
    NOP(4);
    HwGPIOSet(Board_I2C0_SDA0,0);
    NOP(4);
    HwGPIOSet(Board_I2C0_SCL0,0);
}
//产生IIC停止信号
static void IIC_Stop(void)
{
    SDA_OUT();
    HwGPIOSet(Board_I2C0_SCL0,0);
    HwGPIOSet(Board_I2C0_SDA0,0);
    
    NOP(4);
    HwGPIOSet(Board_I2C0_SCL0,1);
    HwGPIOSet(Board_I2C0_SDA0,1);
    NOP(4);
}
static uint8_t IIC_Wait_Ack(void)
{
    uint8_t WaitTime = 0;
    SDA_IN();
    HwGPIOSet(Board_I2C0_SDA0,1);
    NOP(1);
    HwGPIOSet(Board_I2C0_SCL0,1);
    NOP(1);
    while(Board_I2C0_SDA0)
    {
        WaitTime++;
        if(WaitTime > 250)
        {
            IIC_Stop();
            return 1;
        }
    }
    HwGPIOSet(Board_I2C0_SCL0,0);
    return 0;
}
static void IIC_Ack(void)
{
    HwGPIOSet(Board_I2C0_SCL0,0);
    SDA_OUT();
    HwGPIOSet(Board_I2C0_SDA0,0);
    NOP(2);
    HwGPIOSet(Board_I2C0_SCL0,1);
    NOP(2);
    HwGPIOSet(Board_I2C0_SCL0,0);
}
static void IIC_NAck(void)
{
    HwGPIOSet(Board_I2C0_SCL0,0);
    SDA_OUT();
    HwGPIOSet(Board_I2C0_SDA0,1);
    NOP(2);
    HwGPIOSet(Board_I2C0_SCL0,1);
    NOP(2);
    HwGPIOSet(Board_I2C0_SCL0,0);
}
void IIC_Send_Byte(uint8_t TX)
{
    uint8_t i;
    SDA_OUT();
    
    HwGPIOSet(Board_I2C0_SCL0,0);
    for(i=0;i<8;i++)
    {              
        HwGPIOSet(Board_I2C0_SDA0,(TX&0x80)>>7);
        TX<<=1; 	  
        NOP(2);  
        HwGPIOSet(Board_I2C0_SCL0,1);
        NOP(2); 
        HwGPIOSet(Board_I2C0_SCL0,0);	
        NOP(2);
    }	 
}
uint8_t IIC_Read_Byte(uint8_t Ack)
{
    uint8_t i,receive = 0;
    SDA_IN();
    for(i=0;i<8;i++)
    {
        HwGPIOSet(Board_I2C0_SCL0,0);
        NOP(2);
        HwGPIOSet(Board_I2C0_SCL0,1);
        receive<<=1;
        if(Board_I2C0_SDA0)
          receive++;
        NOP(1);
    }
    if(!Ack)
      IIC_NAck();
    else
      IIC_Ack();
    return receive;
}
void IIC_WriteBytes(uint8_t WriteAddr,uint8_t* data,uint8_t dataLength)
{		
      uint8_t i;	
      IIC_Start();  
      
      IIC_Send_Byte(WriteAddr);	    
      IIC_Wait_Ack();
      
      for(i=0;i<dataLength;i++)
      {
              IIC_Send_Byte(data[i]);
              IIC_Wait_Ack();
      }				    	   
      IIC_Stop();
      Delay(1000);	 
}

void IIC_ReadBytes(uint8_t deviceAddr, uint8_t writeAddr,uint8_t* data,uint8_t dataLength)
{		
      uint8_t i;	
      IIC_Start();  

      IIC_Send_Byte(deviceAddr);	   
      IIC_Wait_Ack();
      IIC_Send_Byte(writeAddr);
      IIC_Wait_Ack();
      IIC_Send_Byte(deviceAddr|0X01);		   
      IIC_Wait_Ack();
      
      for(i=0;i<dataLength-1;i++)
      {
              data[i] = IIC_Read_Byte(1);
      }		
      data[dataLength-1] = IIC_Read_Byte(0);	
      IIC_Stop();
      Delay(1000);	 
}

void IIC_Read_One_Byte(uint8_t daddr,uint8_t addr,uint8_t* data)
{				  	  	    																 
    IIC_Start();  
	
    IIC_Send_Byte(daddr);	  
    IIC_Wait_Ack();
    IIC_Send_Byte(addr);
    IIC_Wait_Ack();		 
    IIC_Start();  	 	   
    IIC_Send_Byte(daddr|0X01);			   
    IIC_Wait_Ack();	 
    *data = IIC_Read_Byte(0);		   
    IIC_Stop();	    
}

void IIC_Write_One_Byte(uint8_t daddr,uint8_t addr,uint8_t data)
{				   	  	    																 
    IIC_Start();  
	
    IIC_Send_Byte(daddr);	
    IIC_Wait_Ack();
    IIC_Send_Byte(addr);
    IIC_Wait_Ack();	   	 										  		   
    IIC_Send_Byte(data);   							   
    IIC_Wait_Ack();  		    	   
    IIC_Stop();
    Delay(1000);	 
}