#include "oxygen.h"
#include "hw_uart.h"
#include "string.h"
#include "board.h"
#include "hw_gpio.h"
#include "algorithm.h"
#include "hw_i2c.h"
uint8_t rxbuf[30]={0};

uint32_t aun_ir_buffer[250]; //IR LED sensor data
uint32_t aun_red_buffer[250];    //Red LED sensor data
//uint32_t cn_dx[256];
//uint32_t aun_ir_buffer[250]; //IR LED sensor data
//uint32_t aun_red_buffer[250];    //Red LED sensor data
int32_t n_ir_buffer_length;    //data length
int32_t n_sp02; //SPO2 value
int8_t ch_spo2_valid;   //indicator to show if the SP02 calculation is valid
int32_t n_heart_rate;   //heart rate value
int8_t  ch_hr_valid;    //indicator to show if the heart rate calculation is valid
uint8_t oxygen_time;  //记录数据处理阶段
//uint8_t uch_dummy;
static void max30102_reset(void)
{
      HwI2CSet(0x57,REG_MODE_CONFIG,0x40);
      HwI2CSet(0x57,REG_MODE_CONFIG,0x40);
}
void max30102_init(void)
{
      HwGPIOSet(OXYGEN_POWER_EN,1);
  
      max30102_reset();
      HwI2CSet(0x57,REG_INTR_ENABLE_1,0xc0);	// INTR setting
     
      HwI2CSet(0x57,REG_INTR_ENABLE_2,0x00);
      HwI2CSet(0x57,REG_FIFO_WR_PTR,0x00);  	//FIFO_WR_PTR[4:0]
      HwI2CSet(0x57,REG_OVF_COUNTER,0x00);  	//OVF_COUNTER[4:0]
      HwI2CSet(0x57,REG_FIFO_RD_PTR,0x00);  	//FIFO_RD_PTR[4:0]
      HwI2CSet(0x57,REG_FIFO_CONFIG,0x0f);  	//sample avg = 1, fifo rollover=false, fifo almost full = 17
      HwI2CSet(0x57,REG_MODE_CONFIG,0x03);  	//0x02 for Red only, 0x03 for SpO2 mode 0x07 multimode LED
      HwI2CSet(0x57,REG_SPO2_CONFIG,0x23);  	// SPO2_ADC range = 4096nA, SPO2 sample rate (100 Hz), LED pulseWidth (400uS)  
      HwI2CSet(0x57,REG_LED1_PA,0x24);   	//Choose value for ~ 7mA for LED1
      HwI2CSet(0x57,REG_LED2_PA,0x24);   	// Choose value for ~ 7mA for LED2
      HwI2CSet(0x57,REG_PILOT_PA,0x7f);   	// Choose value for ~ 25mA for Pilot LED
      
}
void max30102_ReadID(void)
{
  
  HwI2CGet(0x57,0xff, rxbuf,1);
  //HwUARTWrite(rxbuf,1); 
}
void oxygen_get_value(void)
{
    uint16_t i;
    uint8_t temp[6];
    //uint32_t un_min, un_max, un_prev_data;
    n_ir_buffer_length = 100;
    for(i=0;i<n_ir_buffer_length;i++)
    {
          HwI2CGet(0x57,REG_FIFO_DATA, temp, 6);
          aun_red_buffer[i] =  (long)((long)((long)temp[0]&0x03)<<16) | (long)temp[1]<<8 | (long)temp[2];    // Combine values to get the actual number
          aun_ir_buffer[i] = (long)((long)((long)temp[3] & 0x03)<<16) |(long)temp[4]<<8 | (long)temp[5];
    }
    //un_prev_data=aun_red_buffer[i];
    //calculate heart rate and SpO2 after first 500 samples (first 5 seconds of samples)
//    rxbuf[1]=(uint8_t)n_ir_buffer_length;
//    rxbuf[2]=n_sp02;
//    rxbuf[3]=ch_spo2_valid;
//    rxbuf[4]=(uint8_t)n_heart_rate;
//    rxbuf[5]=ch_hr_valid;
    maxim_heart_rate_and_oxygen_saturation(aun_ir_buffer, n_ir_buffer_length, aun_red_buffer, &n_sp02, &ch_spo2_valid, &n_heart_rate, &ch_hr_valid); 
    //rxbuf[2*oxygen_time-2]=n_heart_rate;
    //rxbuf[2*oxygen_time-1]=n_sp02;
    rxbuf[0]=n_heart_rate>>24;
    rxbuf[1]=n_heart_rate>>16;
    rxbuf[2]=n_heart_rate>>8;
    rxbuf[3]=n_heart_rate;
    rxbuf[4]=n_sp02>>24;
    rxbuf[5]=n_sp02>>16;
    rxbuf[6]=n_sp02>>8;
    rxbuf[7]=n_sp02;
    //HwUARTPrintf("ch_hr_valid=%d\r\n",ch_hr_valid);
    //HwUARTPrintf("heart_rate=%d\r\n",n_heart_rate);
    //HwUARTPrintf("SPO2=%d\r\n",n_sp02);
}