/******************************************************************************

 @file  sensor_bmp280.c

 @brief Driver for the Bosch BMP280 Pressure Sensor

 Group: WCS, LPC, BTS
 Target Device: CC2650, CC2640, CC1350

 ******************************************************************************
 
 Copyright (c) 2012-2016, Texas Instruments Incorporated
 All rights reserved.

 Redistribution and use in source and binary forms, with or without
 modification, are permitted provided that the following conditions
 are met:

 *  Redistributions of source code must retain the above copyright
    notice, this list of conditions and the following disclaimer.

 *  Redistributions in binary form must reproduce the above copyright
    notice, this list of conditions and the following disclaimer in the
    documentation and/or other materials provided with the distribution.

 *  Neither the name of Texas Instruments Incorporated nor the names of
    its contributors may be used to endorse or promote products derived
    from this software without specific prior written permission.

 THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

 ******************************************************************************
 Release Name: ble_sdk_2_02_01_18
 Release Date: 2016-10-26 15:20:04
 *****************************************************************************/

/* -----------------------------------------------------------------------------------------------
*                                          Includes
* ------------------------------------------------------------------------------------------------
*/
#include "sensor_bmp280.h"
#include "board.h"
#include "stddef.h"
#include "hw_i2c.h"
#include "hw_gpio.h"

/* ------------------------------------------------------------------------------------------------
*                                           Constants and macros
* ------------------------------------------------------------------------------------------------
*/
uint8_t Data[6]={0};
int32_t Temp = 0;
uint32_t Press = 0;
// Sensor I2C address
#define BMP280_I2C_ADDRESS                  0x76

// Registers
#define ADDR_CALIB                          0x88
#define ADDR_PROD_ID                        0xD0
#define ADDR_RESET                          0xE0
#define ADDR_STATUS                         0xF3
#define ADDR_CTRL_MEAS                      0xF4
#define ADDR_CONFIG                         0xF5
#define ADDR_PRESS_MSB                      0xF7
#define ADDR_PRESS_LSB                      0xF8
#define ADDR_PRESS_XLSB                     0xF9
#define ADDR_TEMP_MSB                       0xFA
#define ADDR_TEMP_LSB                       0xFB
#define ADDR_TEMP_XLSB                      0xFC

// Reset values
#define VAL_PROD_ID                         0x58
#define VAL_RESET                           0x00
#define VAL_STATUS                          0x00
#define VAL_CTRL_MEAS                       0x00
#define VAL_CONFIG                          0x00
#define VAL_PRESS_MSB                       0x80
#define VAL_PRESS_LSB                       0x00
#define VAL_TEMP_MSB                        0x80
#define VAL_TEMP_LSB                        0x00

// Test values
#define VAL_RESET_EXECUTE                   0xB6
#define VAL_CTRL_MEAS_TEST                  0x55

// Misc.
#define MEAS_DATA_SIZE                      6
#define CALIB_DATA_SIZE                     24

#define RES_OFF                             0
#define RES_ULTRA_LOW_POWER                 1
#define RES_LOW_POWER                       2
#define RES_STANDARD                        3
#define RES_HIGH                            5
#define RES_ULTRA_HIGH                      6

// Bit fields in CTRL_MEAS register
#define PM_OFF                              0
#define PM_FORCED                           1
#define PM_NORMAL                           3

#define OSRST(v)                            ((v) << 5)
#define OSRSP(v)                            ((v) << 2)

// Sensor selection/deselection
#define SENSOR_SELECT()                     HwGPIOSet(BAROMETER_POWER, 1)
#define SENSOR_DESELECT()                   HwGPIOSet(BAROMETER_POWER, 0)

static bool BMP280_ReadREG(uint8_t addr, uint8_t *pBuf, uint8_t len)
{
    return HwI2CGet(BMP280_I2C_ADDRESS,addr, pBuf, len);
}
static bool BMP280_WriteREG(uint8_t addr, uint8_t *pBuf, uint8_t len)
{
    return HwI2CSet_LenByte(BMP280_I2C_ADDRESS,addr, pBuf, len);
}
/* ------------------------------------------------------------------------------------------------
*                                           Type Definitions
* ------------------------------------------------------------------------------------------------
*/
typedef struct {
	uint16_t dig_T1;
	int16_t dig_T2;
	int16_t dig_T3;
	uint16_t dig_P1;
	int16_t dig_P2;
	int16_t dig_P3;
	int16_t dig_P4;
	int16_t dig_P5;
	int16_t dig_P6;
	int16_t dig_P7;
	int16_t dig_P8;
	int16_t dig_P9;
	int32_t t_fine;
} Bmp280Calibration_t;

/* ------------------------------------------------------------------------------------------------
*                                           Local Functions
* ------------------------------------------------------------------------------------------------
*/

/* ------------------------------------------------------------------------------------------------
*                                           Local Variables
* ------------------------------------------------------------------------------------------------
*/
static uint8_t calData[CALIB_DATA_SIZE];

/**************************************************************************************************
 * @fn          sensorBmp280Init
 *
 * @brief       Initalise the sensor
 *
 * @return      none
 */
void sensorBmp280Init(void)
{
  uint8_t val;
  uint8_t ID;
  uint8_t CTRL = 0xff;
  uint8_t CONFIG = 0x00;
  SENSOR_SELECT();
  // Reset the sensor
  val = VAL_RESET_EXECUTE;
  BMP280_WriteREG(ADDR_RESET, &val, sizeof(val));
  
  // Read and store calibration data
  BMP280_ReadREG( ADDR_CALIB, calData, CALIB_DATA_SIZE);

  BMP280_ReadREG( ADDR_PROD_ID, &ID, sizeof(ID));
  
  BMP280_WriteREG(ADDR_CTRL_MEAS,&CTRL,sizeof(CTRL));
  
  BMP280_WriteREG(ADDR_CONFIG,&CONFIG,sizeof(CONFIG));
}


/**************************************************************************************************
 * @fn          sensorBmp280Enable
 *
 * @brief       Enable/disable measurements
 *
 * @param       enable - flag to turn the snesor on/off
 *
 * @return      none
 */
//void sensorBmp280Enable(bool enable)
//{
//  uint8_t val;
//  
//  if (enable)
//  {
//    // Enable forced mode
//    val = PM_NORMAL | OSRSP(1) | OSRST(1);
//  }
//  else
//  {
//    val = PM_OFF;
//  }
//  BMP280_WriteREG(ADDR_CTRL_MEAS, &val, sizeof(val));
//
//}
#define ST_ERROR_DATA                         0xCC
void BMP280_ErrorData(uint8_t *pBuf, uint8_t n)
{
    while (n > 0) {
        n--;
        pBuf[n] = ST_ERROR_DATA;
    }
}

/**************************************************************************************************
 * @fn          sensorBmp280Read
 *
 * @brief       Read temperature and pressure data
 *
 * @param       data - buffer for temperature and pressure (6 bytes)
 *
 * @return      TRUE if valid data
 */
bool sensorBmp280Read(uint8_t *data)
{
  bool success;
 
  success = BMP280_ReadREG( ADDR_PRESS_MSB, data, MEAS_DATA_SIZE);
  if (!success)
  {
    BMP280_ErrorData(data,MEAS_DATA_SIZE);
  }
  
  return success;
}


/**************************************************************************************************
 * @fn          sensorBmp280Convert
 *
 * @brief       Convert raw data to object and ambience tempertaure
 *
 * @param       data - raw data from sensor
 *
 * @param       temp - converted temperature
 *
 * @param       press - converted pressure
 *
 * @return      none
 **************************************************************************************************/
void sensorBmp280Convert(uint8_t *data, int32_t *temp, uint32_t *press)
{
  int32_t utemp, upress;
  Bmp280Calibration_t *p = (Bmp280Calibration_t *)calData;
	int32_t v_x1_u32r;
	int32_t v_x2_u32r;
	int32_t temperature;
	uint32_t pressure;
  
  // Pressure
  upress = (int32_t)((((uint32_t)(data[0])) << 12) | (((uint32_t)(data[1])) << 4) | ((uint32_t)data[2] >> 4));
  
  // Temperature
  utemp = (int32_t)((( (uint32_t) (data[3])) << 12) |	(((uint32_t)(data[4])) << 4) | ((uint32_t)data[5] >> 4));
  
  // Compensate temperature
	v_x1_u32r  = ((((utemp >> 3) - ((int32_t)p->dig_T1 << 1))) * ((int32_t)p->dig_T2)) >> 11;
	v_x2_u32r  = (((((utemp >> 4) - ((int32_t)p->dig_T1)) * ((utemp >> 4) - ((int32_t)p->dig_T1))) >> 12) * ((int32_t)p->dig_T3)) >> 14;
	p->t_fine = v_x1_u32r + v_x2_u32r;
	temperature  = (p->t_fine * 5 + 128) >> 8;  
  *temp = temperature;

  // Compensate pressure
	v_x1_u32r = (((int32_t)p->t_fine) >> 1) - (int32_t)64000;
	v_x2_u32r = (((v_x1_u32r >> 2) * (v_x1_u32r >> 2)) >> 11) * ((int32_t)p->dig_P6);
	v_x2_u32r = v_x2_u32r + ((v_x1_u32r * ((int32_t)p->dig_P5)) << 1);
	v_x2_u32r = (v_x2_u32r >> 2) + (((int32_t)p->dig_P4) << 16);
	v_x1_u32r = (((p->dig_P3 * (((v_x1_u32r >> 2) * (v_x1_u32r >> 2)) >> 13)) >> 3) + 
      ((((int32_t)p->dig_P2) * v_x1_u32r) >> 1)) >> 18;
	v_x1_u32r = ((((32768+v_x1_u32r)) * ((int32_t)p->dig_P1))	>> 15);
  
	if (v_x1_u32r == 0)
		return; /* Avoid exception caused by division by zero */
  
	pressure = (((uint32_t)(((int32_t)1048576) - upress) - (v_x2_u32r >> 12))) * 3125;
	if (pressure < 0x80000000)
		pressure = (pressure << 1) / ((uint32_t)v_x1_u32r);
	else
		pressure = (pressure / (uint32_t)v_x1_u32r) * 2;
	v_x1_u32r = (((int32_t)p->dig_P9) * ((int32_t)(((pressure >> 3) * (pressure >> 3)) >> 13))) >> 12;
	v_x2_u32r = (((int32_t)(pressure >> 2)) * ((int32_t)p->dig_P8)) >> 13;
	pressure = (uint32_t)((int32_t)pressure + ((v_x1_u32r + v_x2_u32r + p->dig_P7) >> 4));
  
  *press = pressure;
}

/**************************************************************************************************
 * @fn          sensorBmp280Test
 *
 * @brief       Run a sensor self-test
 *
 * @return      TRUE if passed, FALSE if failed
 */
bool sensorBmp280Test(void)
{
  uint8_t val = 0;

  val = VAL_CTRL_MEAS_TEST;
  BMP280_WriteREG(ADDR_CTRL_MEAS, &val, sizeof(val));
  val = 0;
  BMP280_ReadREG(ADDR_CTRL_MEAS, &val, sizeof(val));

   if(val == VAL_CTRL_MEAS_TEST){ 
     return true;
   }
   return false;
}
uint8_t BMP280_GetValue(void)
{
    sensorBmp280Read(Data);
    sensorBmp280Convert(Data,&Temp,&Press);
    return Press;
}
