#include "oxygen.h"
#include "iic2640.h"
#include "hw_uart.h"
#include "string.h"
#include "board.h"
#include "hw_gpio.h"
#include "algorithm.h"
#include <ti/mw/display/Display.h>

extern Display_Handle dispHandle;

uint32_t aun_ir_buffer[500]; //IR LED sensor data
int32_t n_ir_buffer_length;    //data length
uint32_t aun_red_buffer[500];    //Red LED sensor data
int32_t n_sp02; //SPO2 value
int8_t ch_spo2_valid;   //indicator to show if the SP02 calculation is valid
int32_t n_heart_rate;   //heart rate value
int8_t  ch_hr_valid;    //indicator to show if the heart rate calculation is valid
uint8_t uch_dummy;
static void max30102_reset(void)
{
      i2c_write_one_byte(0xae>>1>>1,REG_MODE_CONFIG,0x40);
      i2c_write_one_byte(0xae>>1,REG_MODE_CONFIG,0x40);
}
void max30102_init(void)
{
      max30102_reset();
      i2c_write_one_byte(0xae>>1,REG_INTR_ENABLE_1,0xc0);	// INTR setting
     
      i2c_write_one_byte(0xae>>1,REG_INTR_ENABLE_2,0x00);
      i2c_write_one_byte(0xae>>1,REG_FIFO_WR_PTR,0x00);  	//FIFO_WR_PTR[4:0]
      i2c_write_one_byte(0xae>>1,REG_OVF_COUNTER,0x00);  	//OVF_COUNTER[4:0]
      i2c_write_one_byte(0xae>>1,REG_FIFO_RD_PTR,0x00);  	//FIFO_RD_PTR[4:0]
      i2c_write_one_byte(0xae>>1,REG_FIFO_CONFIG,0x0f);  	//sample avg = 1, fifo rollover=false, fifo almost full = 17
      i2c_write_one_byte(0xae>>1,REG_MODE_CONFIG,0x03);  	//0x02 for Red only, 0x03 for SpO2 mode 0x07 multimode LED
      i2c_write_one_byte(0xae>>1,REG_SPO2_CONFIG,0x27);  	// SPO2_ADC range = 4096nA, SPO2 sample rate (100 Hz), LED pulseWidth (400uS)  
      i2c_write_one_byte(0xae>>1,REG_LED1_PA,0x24);   	//Choose value for ~ 7mA for LED1
      i2c_write_one_byte(0xae>>1,REG_LED2_PA,0x24);   	// Choose value for ~ 7mA for LED2
      i2c_write_one_byte(0xae>>1,REG_PILOT_PA,0x7f);   	// Choose value for ~ 25mA for Pilot LED
}
void max30102_ReadID(void)
{
  uint8_t rxbuf[1]={0};
  i2c_read_one_byte(0xaf>>1,0xff, rxbuf);
  HwUARTWrite(rxbuf,1); 
}
void oxygen_get_value(void)
{
    uint8_t temp[6];
    i2c_read_multi_byte(0xaf>>1,REG_FIFO_DATA, temp, 6);
    HwUARTWrite(temp,6);
}
