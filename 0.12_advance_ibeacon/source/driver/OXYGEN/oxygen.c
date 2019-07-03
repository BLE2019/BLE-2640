#include "oxygen.h"
#include "iic2640.h"
#include "hw_uart.h"
#include "string.h"
#include "board.h"
#include "hw_gpio.h"
#include "algorithm.h"
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
      i2c_write_one_byte(0xae>>1>>1,REG_MODE_CONFIG,0x40);
      i2c_write_one_byte(0xae>>1,REG_MODE_CONFIG,0x40);
}
void max30102_init(void)
{
      HwGPIOSet(Board_LCD_CSN,0);
      max30102_reset();
      i2c_write_one_byte(0xae>>1,REG_INTR_ENABLE_1,0xc0);	// INTR setting
     
      i2c_write_one_byte(0xae>>1,REG_INTR_ENABLE_2,0x00);
      i2c_write_one_byte(0xae>>1,REG_FIFO_WR_PTR,0x00);  	//FIFO_WR_PTR[4:0]
      i2c_write_one_byte(0xae>>1,REG_OVF_COUNTER,0x00);  	//OVF_COUNTER[4:0]
      i2c_write_one_byte(0xae>>1,REG_FIFO_RD_PTR,0x00);  	//FIFO_RD_PTR[4:0]
      i2c_write_one_byte(0xae>>1,REG_FIFO_CONFIG,0x0f);  	//sample avg = 1, fifo rollover=false, fifo almost full = 17
      i2c_write_one_byte(0xae>>1,REG_MODE_CONFIG,0x03);  	//0x02 for Red only, 0x03 for SpO2 mode 0x07 multimode LED
      i2c_write_one_byte(0xae>>1,REG_SPO2_CONFIG,0x23);  	// SPO2_ADC range = 4096nA, SPO2 sample rate (100 Hz), LED pulseWidth (400uS)  
      i2c_write_one_byte(0xae>>1,REG_LED1_PA,0x24);   	//Choose value for ~ 7mA for LED1
      i2c_write_one_byte(0xae>>1,REG_LED2_PA,0x24);   	// Choose value for ~ 7mA for LED2
      i2c_write_one_byte(0xae>>1,REG_PILOT_PA,0x7f);   	// Choose value for ~ 25mA for Pilot LED
}
void max30102_ReadID(void)
{
  
  //i2c_read_one_byte(0xaf>>1,0xff, rxbuf);
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
          i2c_read_multi_byte(0xaf>>1,REG_FIFO_DATA, temp, 6);
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
void oxygen_get_value1(void)
{
    uint8_t i;
    uint8_t temp[6];
    //uint32_t un_min, un_max, un_prev_data;
    n_ir_buffer_length = 100;
    for(i=0;i<n_ir_buffer_length;i++)
    {
        i2c_read_multi_byte(0xaf>>1,REG_FIFO_DATA, temp, 6);
        aun_red_buffer[i] =  (long)((long)((long)temp[0]&0x03)<<16) | (long)temp[1]<<8 | (long)temp[2];    // Combine values to get the actual number
        aun_ir_buffer[i] = (long)((long)((long)temp[3] & 0x03)<<16) |(long)temp[4]<<8 | (long)temp[5];
    }
}
void oxygen_get_value2(void)
{
    uint8_t k,i;
    uint32_t cn_x[256];
    uint32_t un_ir_mean =0;
    uint32_t n_denom,s;
    uint16_t auw_hamm[31]={ 41,    276,    512,    276,     41 }; //Hamm=  long16(512* hamming(5)');
    for (k=0 ; k<n_ir_buffer_length ; k++ ) un_ir_mean += aun_ir_buffer[k] ;
    un_ir_mean =un_ir_mean/n_ir_buffer_length ;
    for (k=0 ; k<n_ir_buffer_length ; k++ )  cn_x[k] =  aun_ir_buffer[k] - un_ir_mean ; 
    
    // 4 pt Moving Average
//    for(k=0; k< BUFFER_SIZE-MA4_SIZE; k++){
//        n_denom= ( cn_x[k]+cn_x[k+1]+ cn_x[k+2]+ cn_x[k+3]);
//        cn_dx[k]=  n_denom/(int32_t)4; 
//    }
    
//    // get difference of smoothed IR signal
//    
//    for( k=0; k<BUFFER_SIZE-MA4_SIZE-1;  k++)
//        cn_dx[k]= (cn_x[k+1]- cn_x[k]);
//
//    // 2-pt Moving Average to an_dx
//    for(k=0; k< BUFFER_SIZE-MA4_SIZE-2; k++){
//        cn_dx[k] =  ( cn_dx[k]+cn_dx[k+1])/2 ;
//    }
//    
//    // hamming window
//    // flip wave form so that we can detect valley with peak detector
//    for ( i=0 ; i<BUFFER_SIZE-HAMMING_SIZE-MA4_SIZE-2 ;i++){
//        s= 0;
//        for( k=i; k<i+ HAMMING_SIZE ;k++){
//            s -= cn_dx[k] *auw_hamm[k-i] ; 
//                     }
//        cn_dx[i]= s/ (int32_t)1146; // divide by sum of auw_hamm 
//    }
}
